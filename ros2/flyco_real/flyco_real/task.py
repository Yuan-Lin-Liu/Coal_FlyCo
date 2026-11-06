"""Terminal task controls independent of Isaac UI / TCP bridge."""
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import Mesh
from std_srvs.srv import Trigger


class GroundHeartbeat(Node):
    def __init__(self):
        super().__init__('ground_heartbeat', automatically_declare_parameters_from_overrides=True)
        if not self.has_parameter('heartbeat') or not self.get_parameter('heartbeat').value:
            raise ValueError('Choose a task action; heartbeat is started by the ground launch')
        self.pub = self.create_publisher(Empty, '/flyco/ground/heartbeat', 1)
        self.timer = self.create_timer(1., lambda: self.pub.publish(Empty()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', choices=['initialize', 'initial-flight', 'enable-planning', 'scan', 'text'])
    parser.add_argument('text', nargs='?')
    args = parser.parse_args(remove_ros_args()[1:])
    if args.action is None:
        from flyco_common.execution import main_for
        main_for(GroundHeartbeat)
        return
    rclpy.init()
    node = Node('flyco_task', automatically_declare_parameters_from_overrides=True)
    try:
        joy = node.create_publisher(Joy, '/joy', 10)
        prompt = node.create_publisher(String, '/query_text', 10)
        data = {'odom': None, 'mesh': False}
        def odom(msg):
            if msg.header.frame_id == 'world':
                data['odom'] = msg
        def mesh(msg):
            data['mesh'] = msg.vertex_num > 0 and msg.face_num > 0
        node.create_subscription(Odometry, '/flyco/odom', odom, qos_profile_sensor_data)
        node.create_subscription(Mesh, '/prediction/predicted_mesh', mesh, 1)
        # Give DDS discovery and initial state time to arrive; scan can await a mesh.
        deadline = time.monotonic() + (30. if args.action == 'scan' else 3.)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() > deadline - 1. and args.action != 'scan':
                break
            if args.action == 'scan' and data['mesh'] and data['odom']:
                break
        msg = data['odom']
        if msg is None or abs(node.get_clock().now().nanoseconds * 1.e-9 -
                (msg.header.stamp.sec + msg.header.stamp.nanosec * 1.e-9)) > .5:
            raise RuntimeError('No fresh /flyco/odom; check LIO, TF and clocks')
        if args.action == 'text':
            if not args.text or not args.text.strip():
                parser.error('text requires a nonempty quoted prompt')
            prompt.publish(String(data=args.text.strip()))
        else:
            if args.action == 'scan' and not data['mesh']:
                raise RuntimeError('No usable predicted mesh received; start ground prediction/send text first')
            button = {'initialize': 4, 'initial-flight': 3, 'enable-planning': 2, 'scan': 5}[args.action]
            message = Joy()
            message.buttons, message.axes = [0] * 12, [0.] * 8
            message.buttons[button] = 1
            message.header.stamp = node.get_clock().now().to_msg()
            if args.action == 'initial-flight':
                client = node.create_client(Trigger, '/flyco/control/takeoff')
                if not client.wait_for_service(timeout_sec=3.):
                    raise RuntimeError('FlyCo takeoff service unavailable; start the control bridge')
                future = client.call_async(Trigger.Request())
                deadline = time.monotonic()+5.
                while not future.done() and time.monotonic()<deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
                result = future.result() if future.done() else None
                node.destroy_client(client)
                if result is None or not result.success:
                    raise RuntimeError(result.message if result else 'Takeoff request timed out')
            # Initial flight belongs to the native mode executor. Publishing
            # the legacy Y pulse as well would queue a second ground-to-air
            # trajectory that can descend after a faster native takeoff.
            if args.action != 'initial-flight':
                joy.publish(message)
        deadline = time.monotonic() + .5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        release = Joy(buttons=[0] * 12, axes=[0.] * 8)
        release.header.stamp = node.get_clock().now().to_msg()
        joy.publish(release)
        print('Sent:', args.action)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
