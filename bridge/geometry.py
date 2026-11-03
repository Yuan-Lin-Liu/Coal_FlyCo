"""Shared camera geometry. No AirSim/NED conversion enters this interface."""
import numpy as np
from scipy.spatial.transform import Rotation

# Optical X right, Y down, Z forward -> body FLU.
OPTICAL_TO_BODY = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def camera_pose(position, attitude, gimbal_degrees, offset=(.125, 0., 0.)):
    body = Rotation.from_quat(attitude).as_matrix()
    # FlyCo's camera command retains the original pitch/yaw signs.
    relative = Rotation.from_euler('xyz', np.asarray(gimbal_degrees) * [1, -1, -1],
                                  degrees=True).as_matrix()
    return np.asarray(position) + body @ offset, body @ relative @ OPTICAL_TO_BODY


def rgbd_points(depth, rgb, position, rotation, intrinsics, stride=4, max_depth=17.):
    fx, fy, cx, cy = intrinsics
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > .1) & (z < max_depth)
    optical = np.column_stack(((u[valid]-cx)*z[valid]/fx,
                               (v[valid]-cy)*z[valid]/fy, z[valid]))
    return (optical @ rotation.T + position).astype('<f4'), rgb[::stride, ::stride][valid]
