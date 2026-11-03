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

import cv2

from perception_msgs.msg import CompressMaskWithPose
from sensor_msgs.msg import CompressedImage

from sam_tracking.utils.utilts import compress_transform


def build_compress_mask_with_pose(mask_rgba, transform, target_size=(640, 480)):
    mask = cv2.resize(mask_rgba, target_size)
    mask[:, :, 3] = cv2.resize(mask_rgba[:, :, 3], target_size, interpolation=cv2.INTER_NEAREST)
    bgra = cv2.cvtColor(mask, cv2.COLOR_RGBA2BGRA)
    success, encoded = cv2.imencode(".png", bgra, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not success:
        return None

    compress_msg = CompressedImage()
    compress_msg.header = transform.header
    compress_msg.format = "rgba8; png compressed"
    compress_msg.data = encoded.tobytes()

    compress_pose = compress_transform(transform)
    compress_mask_with_pose = CompressMaskWithPose()
    compress_mask_with_pose.mask = compress_msg
    compress_mask_with_pose.pose = compress_pose
    compress_mask_with_pose.header.stamp = transform.header.stamp
    compress_mask_with_pose.header.frame_id = "world"
    return compress_mask_with_pose
