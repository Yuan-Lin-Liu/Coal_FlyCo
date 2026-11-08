"""Publish the zero-offset body-to-camera TF used by the Odin deployment."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ZeroCameraTF(Node):
    def __init__(self):
        super().__init__('zero_camera_tf')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'camera_0')
        self.declare_parameter('translation', [0.0, 0.0, 0.0])
        self.declare_parameter('rpy', [0.0, 0.0, 0.0])

        translation = [float(value) for value in self.get_parameter('translation').value]
        rpy = [float(value) for value in self.get_parameter('rpy').value]
        if len(translation) != 3 or len(rpy) != 3:
            raise ValueError('translation and rpy must each contain three values')

        self.broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter('parent_frame').value)
        transform.child_frame_id = str(self.get_parameter('child_frame').value)
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        qx, qy, qz, qw = quaternion_from_rpy(*rpy)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.broadcaster.sendTransform(transform)
        self.get_logger().info(
            'Published static TF %s -> %s: translation=%s rpy=%s'
            % (transform.header.frame_id, transform.child_frame_id, translation, rpy)
        )


def main():
    rclpy.init()
    node = ZeroCameraTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
