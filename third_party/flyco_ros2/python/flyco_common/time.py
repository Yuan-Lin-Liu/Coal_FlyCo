"""Read native ROS 2 timestamps without modifying generated message classes."""
from builtin_interfaces.msg import Time as TimeMessage
from rclpy.duration import Duration
from rclpy.time import Time


def seconds(value):
    if isinstance(value, TimeMessage):
        return value.sec + value.nanosec * 1e-9
    if isinstance(value, (Time, Duration)):
        return value.nanoseconds * 1e-9
    raise TypeError(f"Expected a ROS 2 time or duration, got {type(value).__name__}")


def stamp_delta(first: TimeMessage, second: TimeMessage) -> Duration:
    return Time.from_msg(first) - Time.from_msg(second)
