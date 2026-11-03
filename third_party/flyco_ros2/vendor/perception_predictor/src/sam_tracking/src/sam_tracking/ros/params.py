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

from flyco_common.parameters import parameter
from rclpy.node import Node
from dataclasses import dataclass


from .topics import (
    DEFAULT_CLUSTER_GUIDE_POINTCLOUD_TOPIC,
    DEFAULT_CLUSTER_ACCUMULATED_TARGET_POINTCLOUD_TOPIC,
    DEFAULT_CLUSTER_ENVIRONMENT_POINTCLOUD_TOPIC,
    DEFAULT_CLUSTER_MASK_IMAGE_TOPIC,
    DEFAULT_CLUSTER_PROJECTION_IMAGE_TOPIC,
    DEFAULT_CLUSTER_TARGET_POINTCLOUD_TOPIC,
    DEFAULT_COMPRESSED_MASK_TOPIC,
    DEFAULT_INPUT_DEPTH_MASK_TOPIC,
    DEFAULT_INPUT_IMAGE_TOPIC,
    DEFAULT_INPUT_ODOMETRY_TOPIC,
    DEFAULT_INPUT_POINTCLOUD_TOPIC,
    DEFAULT_LEGACY_PROMPT_POINTS_TOPIC,
    DEFAULT_LEGACY_PROMPT_TEXT_TOPIC,
    DEFAULT_MASK_IMAGE_TOPIC,
    DEFAULT_MASK_WITH_POSE_TOPIC,
    DEFAULT_POSE_TOPIC,
    DEFAULT_PROMPT_POINTS_TOPIC,
    DEFAULT_PROMPT_TEXT_TOPIC,
    DEFAULT_TARGET_POINTCLOUD_TOPIC,
)


@dataclass(frozen=True)
class TrackingRuntimeParams:
    min_height: float
    ckpt_path: str
    cluster_resolution: int
    camera_param: str
    camera_info_topic: str
    query_points_topic: str
    query_text_topic: str
    initial_text_prompt: str
    pc_topic: str
    img_topic: str
    odom_topic: str
    gimbal_topic: str
    airsim_gimbal_use_roll: bool
    depth_mask_topic: str
    airsim_camera_pose_enabled: bool
    tf_static_topic: str
    airsim_odom_frame: str
    airsim_camera_body_frame: str
    airsim_camera_optical_frame: str
    depth_guidance_enabled: bool
    depth_guidance_consistency_threshold: float
    depth_guidance_empty_mask_streak: int
    depth_guidance_reinit_cooldown_frames: int
    depth_guidance_min_mask_area: int
    depth_guidance_bbox_padding: int
    depth_guidance_max_reinit_bbox_area_ratio: float
    depth_guidance_max_reinit_bbox_width_ratio: float
    depth_guidance_max_reinit_bbox_height_ratio: float
    depth_guidance_reinit_on_inconsistency: bool
    depth_guidance_reinit_on_empty_streak: bool
    evf_text_init_enabled: bool
    evf_root: str
    evf_model_version: str
    evf_model_type: str
    evf_precision: str
    evf_device: str
    evf_image_size: int
    evf_semantic_level: bool
    evf_min_mask_area: int
    evf_max_init_points: int
    sam2_samurai_mode: bool
    sam2_depth_score_weight: float
    sam2_kf_score_weight: float
    sam2_memory_bank_geo_score_threshold: float
    cluster_target_pc_topic: str
    cluster_accumulated_target_pc_topic: str
    mask_image_topic: str
    mask_with_pose_topic: str
    compressed_mask_topic: str
    mask_pointcloud_topic: str
    cluster_pointcloud_topic: str
    filtered_pointcloud_topic: str
    pose_topic: str
    cluster_projection_topic: str
    cluster_mask_topic: str
    project_pc_topic: str
    debug_image_topic: str
    debug_pointcloud_topic: str
    cluster_projection_use_accumulated: bool
    cluster_projection_use_hpr: bool
    cluster_incremental_mode: bool
    cluster_input_max_points: int
    cluster_voxel_size: float


@dataclass(frozen=True)
class PromptAdapterParams:
    input_query_points_topic: str
    input_query_text_topic: str
    query_points_topic: str
    query_text_topic: str


def load_tracking_runtime_params(node: Node) -> TrackingRuntimeParams:
    return TrackingRuntimeParams(
        min_height=parameter(node, "min_height", 1.0),
        ckpt_path=parameter(node, "ckpt_path", ""),
        cluster_resolution=parameter(node, "cluster_resolution", 200),
        camera_param=parameter(node, "camera_param", "config/camera.json"),
        camera_info_topic=parameter(node, "camera_info_topic", ""),
        query_points_topic=parameter(node, "query_points_topic", DEFAULT_PROMPT_POINTS_TOPIC),
        query_text_topic=parameter(node, "query_text_topic", DEFAULT_PROMPT_TEXT_TOPIC),
        initial_text_prompt=parameter(node, "initial_text_prompt", ""),
        pc_topic=parameter(node, "pc_topic", DEFAULT_INPUT_POINTCLOUD_TOPIC),
        img_topic=parameter(node, "img_topic", DEFAULT_INPUT_IMAGE_TOPIC),
        odom_topic=parameter(node, "odom_topic", DEFAULT_INPUT_ODOMETRY_TOPIC),
        gimbal_topic=parameter(node, "gimbal_topic", ""),
        airsim_gimbal_use_roll=parameter(node, "airsim_gimbal_use_roll", False),
        depth_mask_topic=parameter(node, "depth_mask_topic", DEFAULT_INPUT_DEPTH_MASK_TOPIC),
        airsim_camera_pose_enabled=parameter(node, "airsim_camera_pose_enabled", False),
        tf_static_topic=parameter(node, "tf_static_topic", "/tf_static"),
        airsim_odom_frame=parameter(node, "airsim_odom_frame", "drone_1/odom_local_enu"),
        airsim_camera_body_frame=parameter(
            node, "airsim_camera_body_frame", "front_center_body/static"
        ),
        airsim_camera_optical_frame=parameter(
            node, "airsim_camera_optical_frame", "front_center_optical/static"
        ),
        depth_guidance_enabled=parameter(node, "depth_guidance_enabled", False),
        depth_guidance_consistency_threshold=parameter(
            node, "depth_guidance_consistency_threshold", 0.7
        ),
        depth_guidance_empty_mask_streak=parameter(node, "depth_guidance_empty_mask_streak", 3),
        depth_guidance_reinit_cooldown_frames=parameter(
            node, "depth_guidance_reinit_cooldown_frames", 8
        ),
        depth_guidance_min_mask_area=parameter(node, "depth_guidance_min_mask_area", 256),
        depth_guidance_bbox_padding=parameter(node, "depth_guidance_bbox_padding", 8),
        depth_guidance_max_reinit_bbox_area_ratio=parameter(
            node, "depth_guidance_max_reinit_bbox_area_ratio", 1.0
        ),
        depth_guidance_max_reinit_bbox_width_ratio=parameter(
            node, "depth_guidance_max_reinit_bbox_width_ratio", 1.0
        ),
        depth_guidance_max_reinit_bbox_height_ratio=parameter(
            node, "depth_guidance_max_reinit_bbox_height_ratio", 1.0
        ),
        depth_guidance_reinit_on_inconsistency=parameter(
            node, "depth_guidance_reinit_on_inconsistency", True
        ),
        depth_guidance_reinit_on_empty_streak=parameter(
            node, "depth_guidance_reinit_on_empty_streak", True
        ),
        evf_text_init_enabled=parameter(node, "evf_text_init_enabled", True),
        evf_root=parameter(node, "evf_root", ""),
        evf_model_version=parameter(node, "evf_model_version", "YxZhang/evf-sam2"),
        evf_model_type=parameter(node, "evf_model_type", "sam2"),
        evf_precision=parameter(node, "evf_precision", "fp16"),
        evf_device=parameter(node, "evf_device", "cuda"),
        evf_image_size=parameter(node, "evf_image_size", 224),
        evf_semantic_level=parameter(node, "evf_semantic_level", False),
        evf_min_mask_area=parameter(node, "evf_min_mask_area", 256),
        evf_max_init_points=parameter(node, "evf_max_init_points", 8),
        sam2_samurai_mode=parameter(node, "sam2_samurai_mode", True),
        sam2_depth_score_weight=parameter(node, "sam2_depth_score_weight", 0.7),
        sam2_kf_score_weight=parameter(node, "sam2_kf_score_weight", 0.15),
        sam2_memory_bank_geo_score_threshold=parameter(
            node, "sam2_memory_bank_geo_score_threshold", 0.0
        ),
        cluster_target_pc_topic=parameter(
            node, "cluster_target_pc_topic", DEFAULT_CLUSTER_TARGET_POINTCLOUD_TOPIC
        ),
        cluster_accumulated_target_pc_topic=parameter(
            node,
            "cluster_accumulated_target_pc_topic",
            DEFAULT_CLUSTER_ACCUMULATED_TARGET_POINTCLOUD_TOPIC,
        ),
        mask_image_topic=parameter(node, "mask_image_topic", DEFAULT_MASK_IMAGE_TOPIC),
        mask_with_pose_topic=parameter(node, "mask_with_pose_topic", DEFAULT_MASK_WITH_POSE_TOPIC),
        compressed_mask_topic=parameter(
            node, "compressed_mask_topic", DEFAULT_COMPRESSED_MASK_TOPIC
        ),
        mask_pointcloud_topic=parameter(
            node, "mask_pointcloud_topic", DEFAULT_TARGET_POINTCLOUD_TOPIC
        ),
        cluster_pointcloud_topic=parameter(
            node, "cluster_pointcloud_topic", DEFAULT_CLUSTER_GUIDE_POINTCLOUD_TOPIC
        ),
        filtered_pointcloud_topic=parameter(
            node, "filtered_pointcloud_topic", DEFAULT_CLUSTER_ENVIRONMENT_POINTCLOUD_TOPIC
        ),
        pose_topic=parameter(node, "pose_topic", DEFAULT_POSE_TOPIC),
        cluster_projection_topic=parameter(
            node, "cluster_projection_topic", DEFAULT_CLUSTER_PROJECTION_IMAGE_TOPIC
        ),
        cluster_mask_topic=parameter(node, "cluster_mask_topic", DEFAULT_CLUSTER_MASK_IMAGE_TOPIC),
        project_pc_topic=parameter(node, "project_pc_topic", "~/project_pc"),
        debug_image_topic=parameter(node, "debug_image_topic", "~/img_debug_topic"),
        debug_pointcloud_topic=parameter(
            node, "debug_pointcloud_topic", "~/debug_pointclouds"
        ),
        cluster_projection_use_accumulated=parameter(
            node, "cluster_projection_use_accumulated", True
        ),
        cluster_projection_use_hpr=parameter(node, "cluster_projection_use_hpr", True),
        cluster_incremental_mode=parameter(node, "cluster_incremental_mode", False),
        cluster_input_max_points=parameter(node, "cluster_input_max_points", 1024),
        cluster_voxel_size=parameter(node, "cluster_voxel_size", 0.2),
    )


def load_prompt_adapter_params(node: Node) -> PromptAdapterParams:
    return PromptAdapterParams(
        input_query_points_topic=parameter(
            node, "input_query_points_topic", DEFAULT_LEGACY_PROMPT_POINTS_TOPIC
        ),
        input_query_text_topic=parameter(
            node, "input_query_text_topic", DEFAULT_LEGACY_PROMPT_TEXT_TOPIC
        ),
        query_points_topic=parameter(node, "query_points_topic", DEFAULT_PROMPT_POINTS_TOPIC),
        query_text_topic=parameter(node, "query_text_topic", DEFAULT_PROMPT_TEXT_TOPIC),
    )
