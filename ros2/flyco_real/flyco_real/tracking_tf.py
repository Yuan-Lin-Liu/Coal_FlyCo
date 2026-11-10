"""Keep FlyCo tracking/cluster algorithms, replace AirSim geometry with timed TF."""
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from flyco_common.execution import main_for
from flyco_common.parameters import parameter
from sam_tracking.ros.tracking_manager import SAMTrackingManager
from .geometry import tf_matrix


class TrackingTF(SAMTrackingManager):
    def load_static_info(self):
        # No default simulator calibration: wait for actual CameraInfo before tracking.
        self.camera_info = CameraInfo(width=640, height=480)
        identity = TransformStamped()
        identity.transform.rotation.w = 1.
        self.static_transforms = {'body2cam': identity, 'cam2body': identity}
        self.real_info_received = False
        self.real_buffer = Buffer(node=self)
        self.real_listener = TransformListener(self.real_buffer, self)

    def __init__(self, name):
        super().__init__(name)
        if self.segmentation_backend.upper() not in ('A', 'SEMANTIC'):
            raise ValueError('Native hardware tracking currently requires the remote semantic backend')
        self.static_rotaion = np.eye(3)
        self.pointcloud_projector.static_rotation = np.eye(3)
        self.cluster_postprocess.static_rotation = np.eye(3)
        self.airsim_camera_pose_enabled = True  # Select common projector's explicit optical-pose path.
        self._airsim_body_static_ready = True
        self.pointcloud_max_stamp_delta = Duration(seconds=parameter(self, 'cloud_sync_slop', .08))
        self.world = parameter(self, 'world_frame', 'world')
        self.camera = parameter(self, 'camera_optical_frame', 'camera_optical_frame')
        self.real_camera_pose = None

    def camera_info_callback(self, msg):
        if msg.header.frame_id != self.camera or msg.k[0] <= 0. or msg.k[4] <= 0.:
            return
        if any(abs(value) > 1.e-8 for value in msg.d):
            return
        self.camera_info = None if not self.real_info_received else self.camera_info
        super().camera_info_callback(msg)
        self.real_info_received = True

    def tracking_timer_callback(self):
        if not self.real_info_received or not self.img_odom_data:
            return
        msg = self.img_odom_data[-1]['img']
        if msg.header.frame_id != self.camera:
            return
        try:
            transform = self.real_buffer.lookup_transform(self.world, self.camera, Time.from_msg(msg.header.stamp))
            self.real_camera_pose = tf_matrix(transform.transform)
        except (TransformException, ValueError) as error:
            self.get_logger().warning('Image-time camera TF unavailable: ' + str(error), throttle_duration_sec=5.)
            self.img_odom_data.clear()
            return
        try:
            super().tracking_timer_callback()
        except (OSError, RuntimeError) as error:
            # Inference link loss must not kill sensor/control executors.
            self.get_logger().error('Semantic inference unavailable: ' + str(error), throttle_duration_sec=5.)
            try:
                self.provider.reset()
            except (OSError, RuntimeError):
                pass  # reset() also clears local session state in its finally block.

    def _build_camera_pose_from_airsim(self, odom_msg, image_stamp=None):
        return self.real_camera_pose

    def destroy_node(self):
        # main_for drains tracking callbacks before teardown. Release only this
        # provider's session so a new tracker can acquire the single-target server.
        try:
            self.provider.reset()
        except Exception as error:
            # An unavailable inference server must not prevent local ROS cleanup.
            self.get_logger().error('Could not release semantic inference session: ' + str(error))
        return super().destroy_node()


def main():
    main_for(TrackingTF, 'sam_tracking')
