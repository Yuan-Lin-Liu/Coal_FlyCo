"""Quaternion helpers with the original xyzw convention."""
import numpy as np
from transforms3d.euler import euler2mat, euler2quat, quat2euler
from transforms3d.quaternions import mat2quat


def translation_matrix(direction):
    matrix = np.eye(4)
    matrix[:3, 3] = direction[:3]
    return matrix


def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < np.finfo(float).eps * 4:
        return np.eye(4)
    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array(
        [
            [1 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], 0],
            [q[0, 1] + q[2, 3], 1 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], 0],
            [q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1 - q[0, 0] - q[1, 1], 0],
            [0, 0, 0, 1],
        ]
    )


def quaternion_from_matrix(matrix):
    q = mat2quat(np.asarray(matrix)[:3, :3])
    return q[[1, 2, 3, 0]]


def euler_from_quaternion(quaternion, axes="sxyz"):
    x, y, z, w = quaternion
    return quat2euler([w, x, y, z], axes=axes)


def euler_matrix(ai, aj, ak, axes="sxyz"):
    matrix = np.eye(4)
    matrix[:3, :3] = euler2mat(ai, aj, ak, axes=axes)
    return matrix


def quaternion_from_euler(ai, aj, ak, axes="sxyz"):
    w, x, y, z = euler2quat(ai, aj, ak, axes=axes)
    return np.array([x, y, z, w])
