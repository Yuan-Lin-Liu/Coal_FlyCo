# ⭐⭐⭐******************************************************************⭐⭐⭐
# Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
#                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
# Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
#                   https://gy920.github.io/
# Date         :    Jun. 2026
# E-mail       :    cfengag at connect dot ust dot hk
#                   shverses at gmail dot com
# Description  :    This file is part of the FlyCo Perception public ROS runtime.
# Copyright    :    Copyright (c) 2026 Chen Feng and Guiyong Zheng.
# License      :    PolyForm Noncommercial License 1.0.0
#                   <https://polyformproject.org/licenses/noncommercial/1.0.0/>
# Project      :    FlyCo: Foundation Model-Empowered Drones for Autonomous 3D Structure Scanning in Open-World Environments
# Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
# ⭐⭐⭐******************************************************************⭐⭐⭐

from flyco_common.execution import spin_node
from flyco_common.parameters import parameter
from flyco_common.time import seconds, stamp_delta
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import message_filters
import numpy as np

import flyco_common.geometry as tft
from cv_bridge import CvBridge
from geometry_msgs.msg import QuaternionStamped
from image_geometry import PinholeCameraModel
from message_filters import ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


class WorldRGBPointCloudPublisher(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name, automatically_declare_parameters_from_overrides=True)
        self._bridge = CvBridge()
        self._camera_model = PinholeCameraModel()
        self._camera_info = None
        self._direction_table = None
        self._body_translation = np.array([0.125, 0.0, 0.0], dtype=np.float64)
        self._body_static_ready = False

        self.image_topic = parameter(self, "image_topic", "/airsim_node/drone_1/front_center/Scene")
        self.depth_topic = parameter(
            self, "depth_topic", "/airsim_node/drone_1/front_center/DepthPerspective"
        )
        self.odom_topic = parameter(self, "odom_topic", "/airsim_node/drone_1/odom_local_enu")
        self.gimbal_topic = parameter(self, "gimbal_topic", "/gimbal_pose")
        self.camera_info_topic = parameter(
            self, "camera_info_topic", "/airsim_node/drone_1/front_center/Scene/camera_info"
        )
        self.tf_static_topic = parameter(self, "tf_static_topic", "/tf_static")
        self.output_topic = parameter(self, "output_topic", "/tracking/debug/world_img_rgb_pcd")
        self.airsim_odom_frame = parameter(self, "airsim_odom_frame", "drone_1/odom_local_enu")
        self.airsim_camera_body_frame = parameter(
            self, "airsim_camera_body_frame", "front_center_body/static"
        )
        self.sync_slop = float(parameter(self, "sync_slop", 0.08))
        self.max_depth = float(parameter(self, "max_depth", 30.0))
        self.min_depth = float(parameter(self, "min_depth", 0.5))
        self.skip_pixels = int(parameter(self, "skip_pixels", 1))
        self.airsim_gimbal_use_roll = bool(parameter(self, "airsim_gimbal_use_roll", False))

        self._img2cam_rotation = np.array(
            [
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ],
            dtype=np.float64,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            self.tf_static_topic,
            self.tf_static_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            self.image_topic,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.odom_sub = message_filters.Subscriber(
            self,
            Odometry,
            self.odom_topic,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.gimbal_sub = message_filters.Subscriber(
            self,
            QuaternionStamped,
            self.gimbal_topic,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.odom_sub, self.gimbal_sub],
            20,
            self.sync_slop,
        )
        self.sync.registerCallback(self.synced_callback)

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.get_logger().info(
            "WorldRGBPointCloudPublisher initialized: image=%s depth=%s odom=%s gimbal=%s output=%s"
            % (
                self.image_topic,
                self.depth_topic,
                self.odom_topic,
                self.gimbal_topic,
                self.output_topic,
            )
        )

    def run(self) -> None:
        spin_node(self)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg
        self._camera_model.fromCameraInfo(msg)
        self._direction_table = self._build_direction_table(msg)

    def tf_static_callback(self, tf_msg: TFMessage) -> None:
        for transform in tf_msg.transforms:
            if transform.header.frame_id != self.airsim_odom_frame:
                continue
            if transform.child_frame_id != self.airsim_camera_body_frame:
                continue
            self._body_translation = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
                dtype=np.float64,
            )
            self._body_static_ready = True

    def _build_direction_table(self, msg: CameraInfo):
        width = int(msg.width)
        height = int(msg.height)
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        u = np.arange(width, dtype=np.float32)
        v = np.arange(height, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        x = (uu - cx) / fx
        y = (vv - cy) / fy
        z = np.ones_like(x, dtype=np.float32)
        direction = np.stack([x, y, z], axis=-1)
        norm = np.linalg.norm(direction, axis=-1, keepdims=True)
        return direction / np.clip(norm, 1e-8, None)

    def _pose_msg_to_matrix(self, position, orientation):
        transform = tft.quaternion_matrix(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        transform[:3, 3] = [position.x, position.y, position.z]
        return transform

    def _build_camera_pose(self, odom_msg: Odometry, gimbal_msg: QuaternionStamped):
        body_to_world = self._pose_msg_to_matrix(
            odom_msg.pose.pose.position, odom_msg.pose.pose.orientation
        )
        body_quat = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ]
        gimbal_quat = [
            gimbal_msg.quaternion.x,
            gimbal_msg.quaternion.y,
            gimbal_msg.quaternion.z,
            gimbal_msg.quaternion.w,
        ]
        roll_body, pitch_body, yaw_body = tft.euler_from_quaternion(body_quat)
        roll_gimbal, pitch_gimbal, yaw_gimbal = tft.euler_from_quaternion(gimbal_quat)
        yaw_gimbal += yaw_body
        pitch_gimbal += pitch_body
        if not self.airsim_gimbal_use_roll:
            roll_gimbal = 0.0
        gimbal_to_body = (
            tft.euler_matrix(0.0, 0.0, -yaw_gimbal)[:3, :3]
            @ tft.euler_matrix(0.0, -pitch_gimbal, 0.0)[:3, :3]
            @ tft.euler_matrix(roll_gimbal, 0.0, 0.0)[:3, :3]
        )
        camera_pose = np.eye(4, dtype=np.float64)
        camera_pose[:3, :3] = body_to_world[:3, :3] @ gimbal_to_body @ self._img2cam_rotation
        camera_pose[:3, 3] = (body_to_world[:3, :3] @ self._body_translation) + body_to_world[:3, 3]
        return camera_pose

    def synced_callback(
        self,
        image_msg: Image,
        depth_msg: Image,
        odom_msg: Odometry,
        gimbal_msg: QuaternionStamped,
    ) -> None:
        if self._camera_info is None or self._direction_table is None:
            self.get_logger().info("Waiting for camera_info", throttle_duration_sec=5.0)
            return
        if not self._body_static_ready:
            self.get_logger().info(
                "Waiting for body extrinsic from /tf_static", throttle_duration_sec=5.0
            )
            return

        rgb = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        if rgb.shape[:2] != depth.shape[:2]:
            self.get_logger().warning(
                "Skip rgb/depth with mismatched size: rgb=%s depth=%s"
                % (rgb.shape[:2], depth.shape[:2]),
                throttle_duration_sec=2.0,
            )
            return

        camera_pose = self._build_camera_pose(odom_msg, gimbal_msg)
        rotation_wc = camera_pose[:3, :3]
        translation_wc = camera_pose[:3, 3]
        step = max(int(self.skip_pixels), 1)
        depth_sampled = depth[::step, ::step]
        rgb_sampled = rgb[::step, ::step]
        direction_sampled = self._direction_table[::step, ::step]
        valid = (
            np.isfinite(depth_sampled)
            & (depth_sampled > self.min_depth)
            & (depth_sampled <= self.max_depth)
        )
        valid_count = int(np.count_nonzero(valid))

        header = Header()
        header.stamp = image_msg.header.stamp
        header.frame_id = "world"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.is_dense = False

        if valid_count > 0:
            depth_valid = depth_sampled[valid].astype(np.float32)
            direction_valid = direction_sampled[valid].astype(np.float32)
            points_cam = direction_valid * depth_valid[:, None]
            points_world = points_cam @ rotation_wc.T.astype(np.float32) + translation_wc.astype(
                np.float32
            )

            rgb_valid = rgb_sampled[valid].astype(np.uint32)
            packed_rgb = (
                (rgb_valid[:, 2] << 16) | (rgb_valid[:, 1] << 8) | rgb_valid[:, 0]
            ).astype(np.uint32)
            packed_rgb_f32 = packed_rgb.view(np.float32)

            cloud_np = np.empty(
                valid_count,
                dtype=[
                    ("x", np.float32),
                    ("y", np.float32),
                    ("z", np.float32),
                    ("rgb", np.float32),
                ],
            )
            cloud_np["x"] = points_world[:, 0]
            cloud_np["y"] = points_world[:, 1]
            cloud_np["z"] = points_world[:, 2]
            cloud_np["rgb"] = packed_rgb_f32

            cloud_msg.width = valid_count
            cloud_msg.row_step = valid_count * cloud_msg.point_step
            cloud_msg.data = cloud_np.tobytes()
        else:
            cloud_msg.width = 0
            cloud_msg.row_step = 0
            cloud_msg.data = b""
        self.cloud_pub.publish(cloud_msg)
        self.get_logger().info(
            "Published world rgb point cloud: points=%d stamp=%.6f"
            % (valid_count, seconds(image_msg.header.stamp)),
            throttle_duration_sec=2.0,
        )

    def run(self) -> None:
        spin_node(self)
