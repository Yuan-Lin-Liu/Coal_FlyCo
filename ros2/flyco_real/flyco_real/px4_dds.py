"""PX4 DDS topic naming and explicit ENU/FLU to NED/FRD conversion."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

ENU_TO_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])


def topic(prefix, direction, name, message_type):
    """Derive the native DDS suffix from the vendored firmware message version."""
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

