"""Normalize LIO odometry/clouds and derive camera feedback from calibrated TF."""
import copy
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, QuaternionStamped
from airsim_ros_pkgs.msg import GimbalAngleEulerCmd
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from .px4_dds import topic, visual_odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from .geometry import pose_matrix, tf_matrix, transform_matrix, planner_gimbal


def set_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge', automatically_declare_parameters_from_overrides=True)
        def param(name, default):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            return self.get_parameter(name).value
        self.world = param('world_frame', 'world')
        self.body = param('body_frame', 'base_link')
        self.camera = param('camera_optical_frame', 'camera_optical_frame')
        self.child_override = param('lio_child_frame', '')
        self.world_from_lio = transform_matrix(param('world_from_lio_translation', [0., 0., 0.]),
                                             param('world_from_lio_quaternion', [0., 0., 0., 1.]))
        self.twist_frame = param('lio_twist_frame', 'child')
        if self.twist_frame not in ('child', 'parent'):
            raise ValueError('lio_twist_frame must be child or parent')
        self.lio_frame = param('lio_parent_frame', 'camera_init')
        self.max_age = param('max_sensor_age', .4)
        self.odom_rate = float(param('odom_publish_rate', 0.))
        if not np.isfinite(self.odom_rate) or self.odom_rate < 0.:
            raise ValueError('odom_publish_rate must be finite and nonnegative')
        self.last_odom_ns = None
        self.voxel = param('cloud_voxel_size', .15)
        self.image_is_rectified = param('image_is_rectified', True)
        self.camera_info = None
        self.rectification = None
        self.calibration_key = None
        self.cv_bridge = CvBridge()
        self.publish_ev = param('publish_external_vision', True)
        ev_frame = param('external_vision_frame', 'FRD')
        if ev_frame not in ('FRD', 'NED'): raise ValueError('external_vision_frame must be FRD or NED')
        self.ev_frame = getattr(VehicleOdometry, 'POSE_FRAME_' + ev_frame)
        self.pose_variances = param('pose_variances', [.04, .04, .09, .01, .01, .02])
        if len(self.pose_variances) != 6 or min(self.pose_variances) <= 0.:
            raise ValueError('Set six measured positive pose variances')
        self.tf_buffer = Buffer(node=self)
        self.listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/flyco/odom', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/flyco/lidar/world', 5)
        self.gimbal_pub = self.create_publisher(QuaternionStamped, '/flyco/gimbal_pose', 10)
        self.gimbal_target = self.create_publisher(QuaternionStamped, '/flyco/gimbal/target_orientation', 10)
        self.ev_pub = self.create_publisher(VehicleOdometry,
            topic(param('px4_topic_prefix', '/fmu'), 'in', 'vehicle_visual_odometry', VehicleOdometry), 1) if self.publish_ev else None
        self.image_pub = self.create_publisher(Image, '/flyco/camera/rgb', 2)
        self.info_pub = self.create_publisher(CameraInfo, '/flyco/camera/info',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, param('odom_topic', '/Odometry'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, param('cloud_topic', '/cloud_registered'), self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Image, param('image_topic', '/camera/color/image_rect'), self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, param('camera_info_topic', '/camera/color/camera_info'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(GimbalAngleEulerCmd, '/flyco/gimbal/legacy_command', self.on_gimbal_command, 10)
        self.get_logger().info('LIO/cloud/camera -> /flyco; all coordinates ENU/FLU, calibration from TF')

    def current(self, stamp):
        age = (self.get_clock().now().nanoseconds - Time.from_msg(stamp).nanoseconds) * 1.e-9
        return -.05 <= age <= self.max_age

    def lookup(self, target, source, stamp):
        if target == source:
            return np.eye(4)
        return tf_matrix(self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp)).transform)

    def on_odom(self, msg):
        if not self.current(msg.header.stamp) or msg.header.frame_id != self.lio_frame:
            return
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self.last_odom_ns is not None and stamp_ns <= self.last_odom_ns:
            return
        if (self.last_odom_ns is not None and self.odom_rate > 0. and
                0 <= stamp_ns - self.last_odom_ns < 1.e9 / self.odom_rate):
            return
        try:
            child = self.child_override or msg.child_frame_id
            if not child:
                raise ValueError('LIO odometry has no child frame; set lio_child_frame explicitly')
            sensor_body = self.lookup(child, self.body, msg.header.stamp)
            sensor_world = self.world_from_lio @ pose_matrix(msg.pose.pose)
            body_world = sensor_world @ sensor_body
            output = copy.deepcopy(msg)
            output.header.frame_id, output.child_frame_id = self.world, self.body
            set_pose(output.pose.pose, body_world)
            linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
            velocity = np.array([linear.x, linear.y, linear.z])
            omega = np.array([angular.x, angular.y, angular.z])
            rotation = sensor_body[:3, :3].T
            if self.twist_frame == 'parent':
                source_rotation = pose_matrix(msg.pose.pose)[:3, :3].T
                velocity, omega = source_rotation @ velocity, source_rotation @ omega
            # v_body = v_sensor + omega x lever arm, both first in sensor frame.
            velocity = rotation @ (velocity + np.cross(omega, sensor_body[:3, 3]))
            omega = rotation @ omega
            output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = map(float, velocity)
            output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = map(float, omega)
            # The source covariance may describe another origin/frame; never copy it blindly.
            output.pose.covariance = [0.] * 36
            for index, variance in enumerate(self.pose_variances):
                output.pose.covariance[index * 7] = variance
            output.twist.covariance = [0.] * 36
            for index in range(6):
                output.twist.covariance[index * 7] = 1.e3
            self.last_odom_ns = stamp_ns
            self.odom_pub.publish(output)
            tf = TransformStamped(header=output.header, child_frame_id=self.body)
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, body_world[:3, 3])
            tf.transform.rotation = output.pose.pose.orientation
            self.broadcaster.sendTransform(tf)
            if self.publish_ev:
                self.ev_pub.publish(visual_odometry(output, self.get_clock().now().nanoseconds // 1000, self.ev_frame))
            # A real gimbal driver must maintain base_link -> optical TF at sensor time.
            # Fixed cameras can supply a calibrated static transform instead.
            camera_body = self.lookup(self.body, self.camera, msg.header.stamp)
            q = planner_gimbal(body_world, body_world @ camera_body)
            feedback = QuaternionStamped(header=output.header)
            feedback.quaternion.x, feedback.quaternion.y, feedback.quaternion.z, feedback.quaternion.w = map(float, q)
            self.gimbal_pub.publish(feedback)
        except (TransformException, ValueError) as error:
            self.get_logger().warning('LIO/camera TF unavailable: ' + str(error), throttle_duration_sec=5.)

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
        if not self.current(msg.header.stamp) or msg.header.frame_id != self.camera or self.camera_info is None:
            return
        if (msg.width, msg.height) != (self.camera_info.width, self.camera_info.height):
            self.get_logger().error('Image resolution differs from CameraInfo', throttle_duration_sec=5.)
            return
        if self.image_is_rectified:
            self.image_pub.publish(msg)
        elif self.rectification is not None:
            raw = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            corrected = cv2.remap(raw, *self.rectification, interpolation=cv2.INTER_LINEAR)
            output = self.cv_bridge.cv2_to_imgmsg(corrected, encoding='rgb8')
            output.header = msg.header
            self.image_pub.publish(output)

    def on_info(self, msg):
        if msg.header.frame_id != self.camera or msg.k[0] <= 0. or msg.k[4] <= 0.:
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
    main_for(SensorBridge)
