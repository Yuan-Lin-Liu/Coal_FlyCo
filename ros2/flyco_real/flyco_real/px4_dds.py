"""PX4 DDS topic naming and explicit ENU/FLU to NED/FRD conversion."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from px4_msgs.msg import VehicleOdometry
from .geometry import pose_matrix

ENU_TO_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])


def topic(prefix, direction, name, message_type):
    """PX4 v1.16 adds _vN only to versioned messages (e.g. VehicleStatus v1)."""
    version = int(getattr(message_type, 'MESSAGE_VERSION', 0))
    suffix = '_v%d' % version if version else ''
    return '/' + prefix.strip('/') + '/' + direction + '/' + name + suffix


def yaw_ned(yaw_enu):
    return math.atan2(math.sin(math.pi / 2. - yaw_enu), math.cos(math.pi / 2. - yaw_enu))


def local_pose_enu(local):
    matrix = np.eye(4)
    matrix[:3, 3] = ENU_TO_NED @ [local.x, local.y, local.z]
    matrix[:3, :3] = Rotation.from_euler('z', yaw_ned(local.heading)).as_matrix()
    return matrix


def visual_odometry(odom, timestamp_us, pose_frame=VehicleOdometry.POSE_FRAME_FRD):
    matrix = pose_matrix(odom.pose.pose)
    msg = VehicleOdometry()
    msg.timestamp = int(timestamp_us)
    msg.timestamp_sample = int(odom.header.stamp.sec) * 1000000 + int(odom.header.stamp.nanosec) // 1000
    msg.pose_frame = pose_frame  # SLAM heading is arbitrary unless explicitly aligned to North.
    msg.position = (ENU_TO_NED @ matrix[:3, 3]).astype(float).tolist()
    q = Rotation.from_matrix(ENU_TO_NED @ matrix[:3, :3] @ FRD_TO_FLU).as_quat()
    msg.q = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
    # The current Odin contract supplies pose; do not claim an unverified velocity.
    msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
    msg.velocity = [math.nan] * 3
    msg.angular_velocity = [math.nan] * 3
    msg.velocity_variance = [math.nan] * 3
    covariance = np.asarray(odom.pose.covariance).reshape(6, 6)
    msg.position_variance = np.diag(ENU_TO_NED @ covariance[:3, :3] @ ENU_TO_NED.T).astype(float).tolist()
    msg.orientation_variance = np.diag(ENU_TO_NED @ covariance[3:, 3:] @ ENU_TO_NED.T).astype(float).tolist()
    msg.reset_counter = 0
    msg.quality = 0  # Odin does not provide a compatible confidence score.
    return msg
