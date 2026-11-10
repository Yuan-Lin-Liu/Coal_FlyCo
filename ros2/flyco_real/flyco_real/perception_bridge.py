"""Point-cloud, camera and gimbal adaptation; odometry is owned by ros2_px4_odometry."""
import copy
from pathlib import Path
import numpy as np
import cv2
import yaml
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, QuaternionStamped
from airsim_ros_pkgs.msg import GimbalAngleEulerCmd
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException
from .geometry import pose_matrix, tf_matrix, transform_matrix, planner_gimbal
from .odin_camera import camera_calibration, image_array


class PerceptionBridge(Node):
    def __init__(self):
        super().__init__('perception_bridge', automatically_declare_parameters_from_overrides=True)
        def param(name, default):
            if not self.has_parameter(name): self.declare_parameter(name, default)
            return self.get_parameter(name).value
        self.world = param('world_frame', 'world')
        self.body = param('body_frame', 'base_link')
        self.camera = param('camera_optical_frame', 'camera_optical_frame')
        self.input_camera = param('input_camera_frame', self.camera)
        self.world_from_lio = transform_matrix(param('world_from_lio_translation', [0., 0., 0.]),
                                             param('world_from_lio_quaternion', [0., 0., 0., 1.]))
        self.lio_frame = param('lio_parent_frame', 'odom')
        self.max_age = param('max_sensor_age', .4)
        self.voxel = param('cloud_voxel_size', .15)
        self.image_is_rectified = param('image_is_rectified', True)
        self.odin_native = param('odin_native_driver', False)
        self.odin_calibration = Path(param('odin_calibration_file',
            # str(Path.home() / '.ros/odin_ros_driver/calib.yaml'))).expanduser() # 11-09
            str(Path.home() / '.ros/odin_ros_driver/calib.yaml'))).expanduser()
        self.camera_info = self.rectification = self.calibration_key = self.body_pose = None
        self._device_time_warned = False
        self.tf_buffer = Buffer(node=self)
        self.listener = TransformListener(self.tf_buffer, self)
        if self.world != self.lio_frame:
            self.world_broadcaster = StaticTransformBroadcaster(self)
            alignment = TransformStamped()
            alignment.header.stamp = self.get_clock().now().to_msg()
            alignment.header.frame_id, alignment.child_frame_id = self.world, self.lio_frame
            alignment.transform.translation.x, alignment.transform.translation.y, alignment.transform.translation.z = map(float, self.world_from_lio[:3, 3])
            q = Rotation.from_matrix(self.world_from_lio[:3, :3]).as_quat()
            alignment.transform.rotation.x, alignment.transform.rotation.y, alignment.transform.rotation.z, alignment.transform.rotation.w = map(float, q)
            self.world_broadcaster.sendTransform(alignment)
        self.cloud_pub = self.create_publisher(PointCloud2, '/flyco/lidar/world', 5)
        self.gimbal_pub = self.create_publisher(QuaternionStamped, '/flyco/gimbal_pose', 10)
        self.gimbal_target = self.create_publisher(QuaternionStamped, '/flyco/gimbal/target_orientation', 10)
        self.image_pub = self.create_publisher(Image, '/flyco/camera/rgb', 2)
        self.info_pub = self.create_publisher(CameraInfo, '/flyco/camera/info',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        native_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE) if self.odin_native else qos_profile_sensor_data
        self.create_subscription(Odometry, '/flyco/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, param('cloud_topic', '/odin1/cloud_slam'), self.on_cloud, native_qos)
        self.create_subscription(Image, param('image_topic', '/odin1/image/undistorted'), self.on_image, native_qos)
        if self.odin_native:
            self.create_timer(1., self.load_odin_calibration)
            self.load_odin_calibration()
        else:
            self.create_subscription(CameraInfo, param('camera_info_topic', '/odin1/camera_info'), self.on_info, native_qos)
        self.create_subscription(GimbalAngleEulerCmd, '/flyco/gimbal/legacy_command', self.on_gimbal_command, 10)
        self.create_timer(1. / 30., self.publish_camera_pose)
        self.get_logger().info('Camera/cloud adaptation; odometry and PX4 forwarding run in ros2_px4_odometry')

    def current(self, stamp):
        age = (self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds) * 1.e-9
        # The unmodified Odin SDK may publish its device-boot clock even when
        # use_host_ros_time is requested in the launch-time control file. A
        # large epoch mismatch identifies that clock domain; receipt freshness
        # is then enforced by the ROS subscription itself. Do not discard live
        # camera/cloud packets solely because their source clock is different.
        if abs(age) > 60.:
            if not self._device_time_warned:
                self.get_logger().warning(
                    'Odin device clock differs from ROS clock; accepting live sensor packets by receipt time')
                self._device_time_warned = True
            return True
        return -.05 <= age <= self.max_age

    def lookup(self, target, source, stamp):
        if target == source: return np.eye(4)
        return tf_matrix(self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp)).transform)

    def on_odom(self, msg):
        if msg.header.frame_id == self.world:
            self.body_pose = msg

    def publish_camera_pose(self):
        msg = self.body_pose
        if msg is None or not self.current(msg.header.stamp): return
        try:
            camera_tf = self.tf_buffer.lookup_transform(self.world, self.camera, Time())
            if not self.current(camera_tf.header.stamp): return
            q = planner_gimbal(pose_matrix(msg.pose.pose), tf_matrix(camera_tf.transform))
            feedback = QuaternionStamped(header=msg.header)
            feedback.quaternion.x, feedback.quaternion.y, feedback.quaternion.z, feedback.quaternion.w = map(float, q)
            self.gimbal_pub.publish(feedback)
        except (TransformException, ValueError) as error:
            self.get_logger().warning('Camera TF unavailable: ' + str(error), throttle_duration_sec=5.)

    def on_cloud(self, msg):
        if not self.current(msg.header.stamp):
            return
        try:
            if msg.header.frame_id == self.lio_frame:
                matrix = self.world_from_lio
            else:
                matrix = self.lookup(self.world, msg.header.frame_id, msg.header.stamp)
            points = np.asarray(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True).tolist(), dtype=np.float32).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1)]
            points = points @ matrix[:3, :3].T + matrix[:3, 3]
            if self.voxel > 0. and len(points):
                _, selected = np.unique(np.floor(points / self.voxel).astype(np.int64), axis=0, return_index=True)
                points = points[selected]
            header = copy.deepcopy(msg.header)
            header.frame_id = self.world
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        except (TransformException, ValueError) as error:
            self.get_logger().warning('Cloud transform unavailable: ' + str(error), throttle_duration_sec=5.)

    def on_image(self, msg):
        if not self.current(msg.header.stamp) or self.camera_info is None:
            return
        if msg.header.frame_id != self.input_camera and not (self.odin_native and not msg.header.frame_id):
            return
        if (msg.width, msg.height) != (self.camera_info.width, self.camera_info.height):
            self.get_logger().error('Image resolution differs from CameraInfo', throttle_duration_sec=5.)
            return
        if self.image_is_rectified:
            output = copy.deepcopy(msg)
        elif self.rectification is not None:
            raw = image_array(msg)
            corrected = cv2.remap(raw, *self.rectification, interpolation=cv2.INTER_LINEAR)
            output = Image(header=copy.deepcopy(msg.header), width=msg.width, height=msg.height,
                encoding=msg.encoding, is_bigendian=False, step=msg.width * 3, data=corrected.tobytes())
        else:
            return
        output.header.frame_id = self.camera
        self.camera_info.header = copy.deepcopy(output.header)
        self.info_pub.publish(self.camera_info)
        self.image_pub.publish(output)

    def load_odin_calibration(self):
        if self.camera_info is not None:
            return
        try:
            document = yaml.safe_load(self.odin_calibration.read_text())
            self.camera_info, self.rectification = camera_calibration(document, self.camera)
            self.get_logger().info('Native Odin calibration loaded: ' + str(self.odin_calibration))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            self.get_logger().warning('Waiting for native Odin calibration: ' + str(error), throttle_duration_sec=5.)

    def on_info(self, msg):
        if msg.header.frame_id != self.input_camera or msg.k[0] <= 0. or msg.k[4] <= 0.:
            return
        key = (msg.width, msg.height, tuple(msg.k), tuple(msg.d), tuple(msg.r), tuple(msg.p), msg.distortion_model)
        if key != self.calibration_key:
            intrinsic = np.array(msg.k).reshape(3, 3)
            projection = np.array(msg.p).reshape(3, 4)
            rectified_intrinsic = projection[:, :3] if projection[0, 0] > 0. else intrinsic
            rotation = np.array(msg.r).reshape(3, 3)
            if not np.any(rotation):
                rotation = np.eye(3)
            if not self.image_is_rectified:
                if msg.distortion_model == 'equidistant':
                    self.rectification = cv2.fisheye.initUndistortRectifyMap(
                        intrinsic, np.array(msg.d), rotation, rectified_intrinsic,
                        (msg.width, msg.height), cv2.CV_32FC1)
                elif msg.distortion_model in ('plumb_bob', 'rational_polynomial', ''):
                    self.rectification = cv2.initUndistortRectifyMap(
                        intrinsic, np.array(msg.d), rotation, rectified_intrinsic,
                        (msg.width, msg.height), cv2.CV_32FC1)
                else:
                    raise ValueError('Unsupported camera distortion model: ' + msg.distortion_model)
            self.camera_info = copy.deepcopy(msg)
            self.camera_info.k = rectified_intrinsic.flatten().tolist()
            self.camera_info.d = [0.] * len(msg.d)
            self.camera_info.r = np.eye(3).flatten().tolist()
            if projection[0, 0] <= 0.:
                self.camera_info.p = np.column_stack((rectified_intrinsic, np.zeros(3))).flatten().tolist()
            self.calibration_key = key
        self.camera_info.header = copy.deepcopy(msg.header)
        self.camera_info.header.frame_id = self.camera
        self.info_pub.publish(self.camera_info)

    def on_gimbal_command(self, msg):
        # Vendor drivers consume this standard body-relative FLU quaternion and
        # supply actual encoder TF. Legacy angle messages never reach hardware.
        if not self.current(msg.header.stamp) or not np.isfinite([msg.roll, msg.pitch, msg.yaw]).all():
            return
        target = QuaternionStamped()
        target.header.stamp, target.header.frame_id = msg.header.stamp, self.body
        q = Rotation.from_euler('xyz', [msg.roll, -msg.pitch, msg.yaw], degrees=True).as_quat()
        target.quaternion.x, target.quaternion.y, target.quaternion.z, target.quaternion.w = map(float, q)
        self.gimbal_target.publish(target)


def main():
    from flyco_common.execution import main_for
    main_for(PerceptionBridge)
