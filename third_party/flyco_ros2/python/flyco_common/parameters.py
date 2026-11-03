"""Parameter declaration shared by the native perception nodes."""
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


def parameter(node: Node, name: str, default=None):
    if not node.has_parameter(name):
        descriptor = ParameterDescriptor(dynamic_typing=default is None)
        node.declare_parameter(name, default, descriptor)
    return node.get_parameter(name).value
