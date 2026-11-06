"""Bridge Odin's live WIWC calibration into the single FlyCo body TF tree."""
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from .geometry import transform_matrix


def sdk_extrinsic(values):
    matrix = np.asarray(values[:16], dtype=float).reshape(4, 4)
    if (not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1.e-5) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1.e-3) or
            abs(np.linalg.det(matrix[:3, :3]) - 1.) > 1.e-3):
        raise ValueError('Invalid Odin WIWC extrinsic matrix')
    return matrix


class OdinAdapter(Node):
    def __init__(self):
        super().__init__('odin_adapter', automatically_declare_parameters_from_overrides=True)
        def param(key, default):
            if not self.has_parameter(key):
                self.declare_parameter(key, default)
            return self.get_parameter(key).value
        self.body = param('body_frame', 'base_link')
        self.imu = param('imu_frame', 'imu')
        self.lidar = param('lidar_frame', 'lidar')
        self.camera = param('camera_optical_frame', 'odin_camera_optical_frame')
        self.mount = transform_matrix(param('body_to_imu_translation', [0., 0., 0.]),
                                      param('body_to_imu_quaternion', [0., 0., 0., 1.]))
        if len({self.body, self.imu, self.lidar, self.camera}) != 4:
            raise ValueError('Odin/body/camera frames must have distinct names')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.calibration = None
        self.lidar_calibration = None
        self.publish_mount()
        self.create_subscription(Odometry, param('extrinsics_topic', '/odin1/wiwc'),
                                 self.on_extrinsics, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info('Waiting for Odin live WIWC calibration; no simulator/factory extrinsic fallback')

    def message(self, parent, child, matrix):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, matrix[:3, 3])
        quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, quaternion)
        return msg

    def publish_mount(self):
        self.broadcaster.sendTransform(self.message(self.body, self.imu, self.mount))

    def on_extrinsics(self, msg):
        try:
            # Odin's WIWC uses these two arrays as row-major transforms, not
            # statistical covariance. T_IL maps LiDAR -> IMU; T_CL maps LiDAR -> optical.
            imu_lidar = sdk_extrinsic(msg.twist.covariance)
            camera_lidar = sdk_extrinsic(msg.pose.covariance)
            imu_camera = imu_lidar @ np.linalg.inv(camera_lidar)
            if self.calibration is not None:
                if (not np.allclose(self.calibration, imu_camera, atol=1.e-3) or
                        not np.allclose(self.lidar_calibration, imu_lidar, atol=1.e-3)):
                    raise RuntimeError('Odin rigid calibration changed; stop and restart the session')
                return
            self.calibration = imu_camera
            self.lidar_calibration = imu_lidar
            self.broadcaster.sendTransform([
                self.message(self.body, self.imu, self.mount),
                self.message(self.imu, self.lidar, imu_lidar),
                self.message(self.imu, self.camera, imu_camera),
            ])
            self.get_logger().info('Odin calibration ready: base_link -> IMU -> LiDAR / optical camera')
        except (ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.)


def main():
    from flyco_common.execution import main_for
    main_for(OdinAdapter)
