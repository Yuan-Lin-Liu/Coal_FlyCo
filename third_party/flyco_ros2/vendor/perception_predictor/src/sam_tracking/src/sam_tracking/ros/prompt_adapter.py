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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32MultiArray as Int32MultiArrayMsg
from std_msgs.msg import String as StringMsg

from .params import load_prompt_adapter_params


class PromptAdapter(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name, automatically_declare_parameters_from_overrides=True)
        self.params = load_prompt_adapter_params(self)
        self.input_query_points_topic = self.resolve_topic_name(
            self.params.input_query_points_topic
        )
        self.input_query_text_topic = self.resolve_topic_name(self.params.input_query_text_topic)
        self.query_points_topic = self.resolve_topic_name(self.params.query_points_topic)
        self.query_text_topic = self.resolve_topic_name(self.params.query_text_topic)
        self.bridge_points = self.input_query_points_topic != self.query_points_topic
        self.bridge_text = self.input_query_text_topic != self.query_text_topic

        self.query_point_sub = self.create_subscription(
            Int32MultiArrayMsg,
            self.input_query_points_topic,
            self.query_point_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.query_text_sub = self.create_subscription(
            StringMsg,
            self.input_query_text_topic,
            self.query_text_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.query_point_pub = (
            self.create_publisher(
                Int32MultiArrayMsg,
                self.query_points_topic,
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            if self.bridge_points
            else None
        )
        self.query_text_pub = (
            self.create_publisher(
                StringMsg,
                self.query_text_topic,
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            if self.bridge_text
            else None
        )

        self.last_points_msg = None
        self.last_text_msg = None

        self.get_logger().info(
            "PromptAdapter initialized: input_points=%s input_text=%s points=%s text=%s"
            % (
                self.input_query_points_topic,
                self.input_query_text_topic,
                self.query_points_topic,
                self.query_text_topic,
            )
        )

    def query_point_callback(self, msg: Int32MultiArrayMsg) -> None:
        self.last_points_msg = msg
        if self.query_point_pub is not None:
            self.query_point_pub.publish(msg)

    def query_text_callback(self, msg: StringMsg) -> None:
        self.last_text_msg = msg
        if self.query_text_pub is not None:
            self.query_text_pub.publish(msg)

    def run(self) -> None:
        spin_node(self)
