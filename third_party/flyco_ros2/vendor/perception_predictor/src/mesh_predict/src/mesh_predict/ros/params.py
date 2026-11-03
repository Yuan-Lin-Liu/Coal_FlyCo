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
from typing import Any, Dict, Optional


from mesh_predict.runtime.resources import (
    discover_package_root,
    load_optional_yaml,
    resolve_existing_file,
)


@dataclass
class OfficialNodeParams:
    """Resolved ROS parameters for the official runtime node."""

    runtime_config_path: str
    mesh_pred_ckpt_path: Optional[str]
    filtered_pc_topic: str
    camera_param: Optional[str]
    camera_info_topic: str
    pc_topic: str
    img_pose_topic: str
    text_prompt_topic: str
    predicted_pointcloud_topic: str
    partial_pointcloud_topic: str
    predicted_mesh_topic: str
    mesh_vis_topic: str
    predicted_input_imgs_topic: str
    debug_dump_dir: str
    debug_dump_once: bool
    min_height: float
    mesh_min_z_offset: float
    keep_largest_mesh_component: bool
    nksr_scale: float
    recall_threshold: float
    recall_distance_threshold: float
    pose_alignment_mode: str
    intrinsics_mode: str = "training_legacy"
    prediction_outlier_filter: bool = False


def _profile_value(profile: Dict[str, Any], key: str, default):
    return profile.get(key, default)


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    return bool(value)


def load_node_params(node: Node) -> OfficialNodeParams:
    """Load ROS parameters and apply optional profile defaults."""

    profile_config_path = parameter(node, "profile_config_path", "")
    profile = load_optional_yaml(profile_config_path)
    profile_package_root = (
        discover_package_root(profile_config_path) if profile_config_path else None
    )
    camera_param = parameter(node, "camera_param", _profile_value(profile, "camera_param", None))
    if camera_param:
        camera_param = resolve_existing_file(
            "camera_param", camera_param, package_root=profile_package_root
        )

    intrinsics_mode = str(
        parameter(
            node, "intrinsics_mode", _profile_value(profile, "intrinsics_mode", "training_legacy")
        )
    )
    if intrinsics_mode not in ("camera_info", "training_legacy"):
        raise ValueError("Unknown intrinsics_mode: " + intrinsics_mode)

    return OfficialNodeParams(
        runtime_config_path=parameter(node, "runtime_config_path", None),
        mesh_pred_ckpt_path=parameter(node, "mesh_pred_ckpt_path", None),
        filtered_pc_topic=parameter(node, "filtered_pc_topic", ""),
        camera_param=camera_param,
        camera_info_topic=parameter(
            node, "camera_info_topic", _profile_value(profile, "camera_info_topic", "")
        ),
        pc_topic=parameter(
            node, "pc_topic", _profile_value(profile, "pc_topic", "/tracking/target_pc")
        ),
        img_pose_topic=parameter(
            node,
            "img_pose_topic",
            _profile_value(profile, "img_pose_topic", "/sam_tracking/compress_mask_and_transform"),
        ),
        text_prompt_topic=parameter(
            node, "text_prompt_topic", _profile_value(profile, "text_prompt_topic", "/text_prompt")
        ),
        predicted_pointcloud_topic=parameter(
            node,
            "predicted_pointcloud_topic",
            _profile_value(profile, "predicted_pointcloud_topic", "mesh_pred/pred_pc"),
        ),
        partial_pointcloud_topic=parameter(
            node,
            "partial_pointcloud_topic",
            _profile_value(profile, "partial_pointcloud_topic", "mesh_pred/partial_pc"),
        ),
        predicted_mesh_topic=parameter(
            node,
            "predicted_mesh_topic",
            _profile_value(profile, "predicted_mesh_topic", "/prediction/predicted_mesh"),
        ),
        mesh_vis_topic=parameter(
            node,
            "mesh_vis_topic",
            _profile_value(profile, "mesh_vis_topic", "/prediction/mesh_vis"),
        ),
        predicted_input_imgs_topic=parameter(
            node,
            "predicted_input_imgs_topic",
            _profile_value(profile, "predicted_input_imgs_topic", "/prediction/input_image"),
        ),
        debug_dump_dir=str(
            parameter(node, "debug_dump_dir", _profile_value(profile, "debug_dump_dir", ""))
        ),
        debug_dump_once=_as_bool(
            parameter(node, "debug_dump_once", _profile_value(profile, "debug_dump_once", False))
        ),
        min_height=float(parameter(node, "min_height", _profile_value(profile, "min_height", 1.0))),
        mesh_min_z_offset=float(
            parameter(node, "mesh_min_z_offset", _profile_value(profile, "mesh_min_z_offset", 0.3))
        ),
        keep_largest_mesh_component=_as_bool(
            parameter(
                node,
                "keep_largest_mesh_component",
                _profile_value(profile, "keep_largest_mesh_component", True),
            )
        ),
        nksr_scale=float(parameter(node, "nksr_scale", _profile_value(profile, "nksr_scale", 0.2))),
        recall_threshold=float(
            parameter(node, "recall_threshold", _profile_value(profile, "recall_threshold", 0.9))
        ),
        recall_distance_threshold=float(
            parameter(
                node,
                "recall_distance_threshold",
                _profile_value(profile, "recall_distance_threshold", 0.1),
            )
        ),
        pose_alignment_mode=str(
            parameter(
                node,
                "pose_alignment_mode",
                _profile_value(profile, "pose_alignment_mode", "legacy_ros"),
            )
        ),
        intrinsics_mode=intrinsics_mode,
        prediction_outlier_filter=_as_bool(
            parameter(
                node,
                "prediction_outlier_filter",
                _profile_value(profile, "prediction_outlier_filter", False),
            )
        ),
    )
