"""Validation shared by the live PX4 adapter and its transport tests."""
import math
import numpy as np


def fresh(stamp, now, timeout):
    return stamp is not None and -.05 <= now - stamp <= timeout


def validate_command(position, velocity, acceleration, yaw, yaw_rate, measured,
                     bounds_min, bounds_max, max_speed, max_acceleration,
                     max_yaw_rate, max_position_error):
    arrays = [np.asarray(v, float) for v in (position, velocity, acceleration, measured)]
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in arrays):
        return 'nonfinite command or odometry'
    position, velocity, acceleration, measured = arrays
    if not math.isfinite(yaw) or not math.isfinite(yaw_rate):
        return 'nonfinite yaw'
    if np.any(position < bounds_min) or np.any(position > bounds_max):
        return 'command outside configured flight volume'
    if np.linalg.norm(velocity) > max_speed:
        return 'velocity limit'
    if np.linalg.norm(acceleration) > max_acceleration:
        return 'acceleration limit'
    if abs(yaw_rate) > max_yaw_rate:
        return 'yaw rate limit'
    if np.linalg.norm(position - measured) > max_position_error:
        return 'position tracking error'
    return ''
