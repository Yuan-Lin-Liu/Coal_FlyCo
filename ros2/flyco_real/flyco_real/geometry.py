"""ROS ENU / FLU geometry; px4_dds performs the explicit PX4 frame conversion."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def transform_matrix(translation, quaternion):
    translation, quaternion = np.asarray(translation, float), np.asarray(quaternion, float)
    if (translation.shape != (3,) or quaternion.shape != (4,) or
            not np.isfinite(translation).all() or not np.isfinite(quaternion).all() or
            abs(np.linalg.norm(quaternion) - 1.) > .05):
        raise ValueError('Expected finite translation and a unit XYZW quaternion')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = translation
    return matrix


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return transform_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def tf_matrix(transform):
    p, q = transform.translation, transform.rotation
    return transform_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def yaw_of(matrix):
    return math.atan2(matrix[1, 0], matrix[0, 0])


def world_alignment(planning_pose, px4_pose):
    """Fix the yaw and origin between two gravity-aligned local ENU frames."""
    delta = yaw_of(px4_pose) - yaw_of(planning_pose)
    rotation = Rotation.from_euler('z', delta).as_matrix()
    return rotation, px4_pose[:3, 3] - rotation @ planning_pose[:3, 3], delta


def planner_gimbal(body_to_world, optical_to_world):
    # FlyCo's pitch is elevation (positive up), unlike optical/FLU Euler pitch.
    # Preserve the actual camera view direction without any simulator signs.
    forward_body = body_to_world[:3, :3].T @ optical_to_world[:3, 2]
    elevation = math.atan2(forward_body[2], np.hypot(*forward_body[:2]))
    azimuth = math.atan2(forward_body[1], forward_body[0])
    return Rotation.from_euler('xyz', [0., elevation, azimuth]).as_quat()
