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

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2 as PointCloud2Msg
from visualization_msgs.msg import Marker

from quadrotor_msgs.msg import Mesh

from mesh_predict.ros.messages import (
    build_mesh_marker,
    build_mesh_message,
    build_xyz_pointcloud,
)


class MeshPredictPublishers:
    """ROS publisher bundle for official runtime outputs."""

    def __init__(self, node: Node, params):
        self.node = node
        self.params = params
        self.bridge = CvBridge()

        self.mesh_pub = self.node.create_publisher(
            Mesh,
            self.params.predicted_mesh_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.vis_mesh_pub = self.node.create_publisher(
            Marker,
            self.params.mesh_vis_topic,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.predict_pc_pub = self.node.create_publisher(
            PointCloud2Msg,
            self.params.predicted_pointcloud_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.pure_predict_pc_pub = self.node.create_publisher(
            PointCloud2Msg,
            "~/pure_predicted_pointcloud",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.debug_pc_pub = self.node.create_publisher(
            PointCloud2Msg,
            "~/debug_pointcloud",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.partial_pc_pub = self.node.create_publisher(
            PointCloud2Msg,
            self.params.partial_pointcloud_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.debug_img_pub = self.node.create_publisher(
            Image,
            self.params.predicted_input_imgs_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

    def publish_pointcloud_prediction(self, result):
        self.predict_pc_pub.publish(
            build_xyz_pointcloud(
                result["predicted_pointcloud"], stamp=self.node.get_clock().now().to_msg()
            )
        )
        self.pure_predict_pc_pub.publish(
            build_xyz_pointcloud(
                result["pure_predicted_pointcloud"], stamp=self.node.get_clock().now().to_msg()
            )
        )
        self.publish_debug_image(result["debug_image"])

    def publish_mesh_prediction(self, result):
        self.partial_pc_pub.publish(
            build_xyz_pointcloud(
                result["partial_pointcloud"], stamp=self.node.get_clock().now().to_msg()
            )
        )
        self.publish_mesh(result["mesh_vertices"], result["mesh_faces"])
        self.publish_mesh_marker(result["mesh_vertices"], result["mesh_faces"])
        self.debug_pc_pub.publish(
            build_xyz_pointcloud(
                result["debug_pointcloud"], stamp=self.node.get_clock().now().to_msg()
            )
        )

    def publish_mesh_marker(self, mesh_v, mesh_f):
        mesh_marker = build_mesh_marker(mesh_v, mesh_f, stamp=self.node.get_clock().now().to_msg())
        mesh_marker.action = Marker.ADD if mesh_marker.points else Marker.DELETE
        self.vis_mesh_pub.publish(mesh_marker)

    def publish_debug_image(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img_msg = self.bridge.cv2_to_imgmsg(img, "8UC3")
        self.debug_img_pub.publish(img_msg)

    def publish_mesh(self, mesh_v, mesh_f):
        self.mesh_pub.publish(build_mesh_message(mesh_v, mesh_f))
