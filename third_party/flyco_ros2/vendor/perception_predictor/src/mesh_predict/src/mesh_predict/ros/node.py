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
from flyco_common.time import seconds, stamp_delta
from mesh_predict.ros.params import load_node_params
from mesh_predict.runtime.resources import load_official_runtime_spec
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from collections import deque
from threading import Lock

import numpy as np

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import CameraInfo, PointCloud2 as PointCloud2Msg
from std_msgs.msg import Bool, Float32, String as StringMsg

from perception_msgs.msg import CompressMaskWithPose

from mesh_predict.core.orchestrator import (
    MeshPredictOrchestrator,
    format_prediction_status,
    format_waiting_reason,
)
from mesh_predict.ros.params import OfficialNodeParams
from mesh_predict.ros.publishers import MeshPredictPublishers
from mesh_predict.runtime.resources import OfficialRuntimeSpec


class MeshPredictNode(Node):
    """Official ROS node wrapper around the runtime orchestrator."""

    def __init__(self, node_name="mesh_predict_node", params=None, runtime_spec=None):
        super().__init__(node_name, automatically_declare_parameters_from_overrides=True)
        if params is None:
            params = load_node_params(self)
        if runtime_spec is None:
            runtime_spec = load_official_runtime_spec(
                params.runtime_config_path, params.mesh_pred_ckpt_path
            )
        self.params = params
        if not 0 <= params.recall_threshold <= 1:
            raise ValueError("recall_threshold must be a Flight Coverage fraction from 0 to 1")
        self.runtime_spec = runtime_spec
        self.get_logger().warning(
            "MeshPredict runtime config=%s model config=%s mesh checkpoint=%s intrinsics_mode=%s prediction_outlier_filter=%s"
            % (
                runtime_spec.runtime_config_path,
                runtime_spec.model_config_path,
                runtime_spec.mesh_pred_checkpoint,
                params.intrinsics_mode,
                params.prediction_outlier_filter,
            )
        )
        self.outputs = MeshPredictPublishers(self, params)
        self.orchestrator = MeshPredictOrchestrator(
            runtime_spec=runtime_spec,
            min_height=params.min_height,
            mesh_min_z_offset=params.mesh_min_z_offset,
            keep_largest_component=params.keep_largest_mesh_component,
            nksr_scale=params.nksr_scale,
            pose_alignment_mode=params.pose_alignment_mode,
            debug_dump_dir=params.debug_dump_dir,
            debug_dump_once=params.debug_dump_once,
            device="cuda",
            intrinsics_mode=params.intrinsics_mode,
            prediction_outlier_filter=params.prediction_outlier_filter,
            observation_voxel_size=(self.get_parameter("observation_voxel_size").value
                if self.has_parameter("observation_voxel_size") else 0.6),
        )

        self.pending_point_messages = deque()
        self.pending_img_pose_messages = deque()
        self._pc_prediction_running = False
        self._mesh_frozen = False
        self._mesh_published = False
        self._mesh_update_lock = Lock()
        self.flight_coverage_sub = self.create_subscription(
            Float32,
            "/planning/flight_coverage",
            self.flight_coverage_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.flight_end_sub = self.create_subscription(
            Bool,
            "/flight_end",
            self.flight_end_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.camera_info_sub = None
        if self.params.camera_info_topic:
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                self.params.camera_info_topic,
                self.camera_info_callback,
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2Msg,
            self.params.pc_topic,
            self.pointcloud_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.compress_img_sub = self.create_subscription(
            CompressMaskWithPose,
            self.params.img_pose_topic,
            self.compress_image_transform_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.text_prompt_sub = self.create_subscription(
            StringMsg,
            self.params.text_prompt_topic,
            self.text_prompt_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.filtered_pc_sub = None
        if self.params.filtered_pc_topic:
            self.filtered_pc_sub = self.create_subscription(
                PointCloud2Msg,
                self.params.filtered_pc_topic,
                self.filtered_pc_callback,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        self.img_pose_process_timer = self.create_timer(
            seconds(Duration(seconds=0.01)),
            self.img_pose_process_timer_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.pointcloud_process_timer = self.create_timer(
            seconds(Duration(seconds=0.01)),
            self.pointcloud_process_timer_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.mesh_predict_timer = self.create_timer(
            seconds(Duration(seconds=2)),
            self.mesh_predict_timer_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.pc_predict_timer = self.create_timer(
            seconds(Duration(seconds=0.5)),
            self.pc_predict_timer_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.prediction_status_timer = self.create_timer(
            seconds(Duration(seconds=2)),
            self.prediction_status_timer_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.get_logger().info(
            f"Prediction inputs: pointcloud={params.pc_topic} "
            f"image_pose={params.img_pose_topic} text={params.text_prompt_topic}"
        )
        if params.intrinsics_mode == "camera_info" and params.camera_info_topic:
            self.get_logger().info("Waiting for live CameraInfo on " + params.camera_info_topic)
        else:
            self.load_static_camera_info()
        self.get_logger().warning("MeshPredictNode Init Done")

    def load_static_camera_info(self):
        camera_param_file = self.params.camera_param or self.runtime_spec.camera_param_default
        self.get_logger().info(f"Loading camera parameters from: {camera_param_file}")
        try:
            self.orchestrator.load_static_camera_info(camera_param_file)
            self.get_logger().info("Camera parameters loaded successfully.")
        except Exception as exc:
            self.get_logger().error(f"Failed to load camera parameters: {exc}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.orchestrator.update_camera_info(msg)

    def pointcloud_callback(self, msg: PointCloud2Msg) -> None:
        self.pending_point_messages.append(msg)

    def compress_image_transform_callback(self, msg: CompressMaskWithPose) -> None:
        self.pending_img_pose_messages.append(msg)

    def text_prompt_callback(self, msg: StringMsg) -> None:
        self.orchestrator.set_text_prompt(msg.data)

    def flight_coverage_callback(self, msg: Float32) -> None:
        coverage = msg.data
        if not 0 <= coverage <= 1:
            return
        with self._mesh_update_lock:
            # Compare at ROS Float32 precision so exactly 0.9 reaches a 0.9 threshold.
            if (
                self._mesh_published
                and not self._mesh_frozen
                and coverage >= float(np.float32(self.params.recall_threshold))
            ):
                self._mesh_frozen = True
                self.get_logger().warning(
                    "Flight Coverage %.2f%% reached %.2f%%; freezing mesh updates"
                    % (coverage * 100, self.params.recall_threshold * 100)
                )

    def flight_end_callback(self, msg: Bool) -> None:
        if not msg.data:
            with self._mesh_update_lock:
                self._mesh_frozen = False

    def filtered_pc_callback(self, ros_pc: PointCloud2Msg) -> None:
        try:
            gen = point_cloud2.read_points(
                ros_pc, field_names=("x", "y", "z"), skip_nans=True
            ).tolist()
            pc_np = np.array(list(gen))
            self.orchestrator.append_filtered_pointcloud(pc_np)
            self.get_logger().info(f"Received filtered point cloud with {len(pc_np)} points.")
        except Exception as exc:
            self.get_logger().error(f"Error processing filtered point cloud: {exc}")

    def img_pose_process_timer_callback(self) -> None:
        if not self.pending_img_pose_messages:
            return

        img_pose_msg_list = list(self.pending_img_pose_messages)
        self.pending_img_pose_messages.clear()
        for msg in img_pose_msg_list:
            self.orchestrator.append_image_pose_message(msg)

    def pointcloud_process_timer_callback(self) -> None:
        if len(self.pending_point_messages) < 5:
            return

        point_msg_list = list(self.pending_point_messages)[-3:]
        self.pending_point_messages.clear()
        self.orchestrator.append_pointcloud_messages(point_msg_list)

    def prediction_status_timer_callback(self) -> None:
        self.get_logger().info(
            format_prediction_status(self.orchestrator)
            + f" pending_cloud_messages={len(self.pending_point_messages)}/5"
            + f" pending_image_messages={len(self.pending_img_pose_messages)}"
            + f" mesh_frozen={self._mesh_frozen}"
        )

    def pc_predict_timer_callback(self) -> None:
        if self._pc_prediction_running:
            self.get_logger().debug("Skipping point cloud prediction while previous run is active")
            return

        self._pc_prediction_running = True
        try:
            result = self.orchestrator.run_pointcloud_prediction()
            if result is None:
                self.get_logger().debug(format_waiting_reason(self.orchestrator))
                return

            self.outputs.publish_pointcloud_prediction(result)
            timings = result.get("timings", {})
            if self.params.prediction_outlier_filter:
                self.get_logger().info(
                    "Prediction SOR neighbors=16 std_ratio=2.5 max_removed_ratio=0.05 input=%s kept=%s removed=%s time=%.3fs"
                    % (
                        timings.get("outlier_input_points", "?"),
                        timings.get("outlier_kept_points", "?"),
                        timings.get("outlier_removed_points", "?"),
                        timings.get("outlier", -1.0),
                    )
                )
            if result.get("debug_dump_path"):
                self.get_logger().warning(
                    "Pointcloud debug dump saved to %s" % (result["debug_dump_path"],)
                )
            self.get_logger().info(
                "PC predict timing total=%.3fs batch=%.3fs to_device=%.3fs infer=%.3fs decode=%.3fs post=%.3fs input_pts=%s partial_pts=%s pred_pts=%s"
                % (
                    timings.get("total", -1.0),
                    timings.get("batch", -1.0),
                    timings.get("to_device", -1.0),
                    timings.get("infer", -1.0),
                    timings.get("decode", -1.0),
                    timings.get("post", -1.0),
                    timings.get("input_points", "?"),
                    timings.get("partial_points", "?"),
                    timings.get("predicted_points", "?"),
                )
            )
        finally:
            self._pc_prediction_running = False

    def mesh_predict_timer_callback(self) -> None:
        if self._mesh_frozen:
            return
        result = self.orchestrator.run_mesh_prediction()
        if result is None:
            self.get_logger().debug("Waiting for mesh prediction metadata...")
            return

        if result["used_fallback"]:
            self.get_logger().warning(
                "filtered point cloud is unavailable; falling back to predicted surface input",
                once=True,
            )
        with self._mesh_update_lock:
            if self._mesh_frozen:
                return
            self.outputs.publish_mesh_prediction(result)
            self._mesh_published = True
        timings = result.get("timings", {})
        self.get_logger().info(
            "Mesh predict timing total=%.3fs input=%.3fs surface=%.3fs pred_mesh=%.3fs post=%.3fs filtered_pts=%s partial_pts=%s surface_pts=%s vertices=%s faces=%s"
            % (
                timings.get("total", -1.0),
                timings.get("input", -1.0),
                timings.get("surface", -1.0),
                timings.get("pred_mesh", -1.0),
                timings.get("post", -1.0),
                timings.get("filtered_input_points", "?"),
                timings.get("partial_points", "?"),
                timings.get("surface_points", "?"),
                timings.get("mesh_vertices", "?"),
                timings.get("mesh_faces", "?"),
            )
        )
        self.get_logger().info("Mesh publish Done")

    def run(self):
        spin_node(self)
