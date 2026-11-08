"""Terminal task nodes for FlyCo; registration belongs to the persistent mode node."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import Mesh
from px4_msgs.msg import VehicleCommand
from std_srvs.srv import Trigger, SetBool
from .px4_dds import topic


CONTROL_ACTIONS = {
    'enable': ('enable', SetBool, True),
    'disable': ('enable', SetBool, False),
    'takeoff': ('takeoff', Trigger, None),
    'initial-flight': ('takeoff', Trigger, None),
    'hold': ('hold', Trigger, None),
    'resume': ('resume', Trigger, None),
    'land': ('land', Trigger, None),
}


class GroundHeartbeat(Node):
    def __init__(self):
        super().__init__('ground_heartbeat', automatically_declare_parameters_from_overrides=True)
        if not self.has_parameter('heartbeat') or not self.get_parameter('heartbeat').value:
            raise ValueError('Choose a task action; heartbeat is started by the ground launch')
        self.pub = self.create_publisher(Empty, '/flyco/ground/heartbeat', 1)
        self.timer = self.create_timer(1., lambda: self.pub.publish(Empty()))


def wait_for(node, check, timeout, message):
    deadline = time.monotonic()+timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if check():
            return
    raise RuntimeError(message)


def control_task(node, action):
    name, kind, data = CONTROL_ACTIONS[action]
    client = node.create_client(kind, '/flyco/control/'+name)
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Start the FlyCo mode node first: /flyco/control/'+name+' unavailable')
        request = kind.Request() if data is None else kind.Request(data=data)
        future = client.call_async(request)
        wait_for(node, future.done, 5., action+' request timed out')
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(result.message if result else action+' returned no response')
        print(action+': '+result.message)
    finally:
        node.destroy_client(client)


def mode_task(node, action):
    status = {}
    last_rx = [0.]
    def update(msg):
        status.update(json.loads(msg.data))
        last_rx[0] = time.monotonic()
    node.create_subscription(String, '/flyco/control/status', update,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait_for(node, lambda: bool(status), 5., 'No FlyCo status; start the mode node first')
    if action == 'status':
        print(json.dumps(status, ensure_ascii=False, indent=2))
        return
    if status.get('registered_mode_id') is None:
        raise RuntimeError('FlyCo mode is not registered; start mode.launch.py first')
    if action == 'arm' and status.get('mode') != 'FlyCo':
        raise RuntimeError('Select FlyCo before arming: task.py select-mode')
    if action == 'arm' and status.get('armed'):
        raise RuntimeError('Already armed')
    if not node.has_parameter('px4_topic_prefix'):
        node.declare_parameter('px4_topic_prefix', '/fmu')
    prefix = node.get_parameter('px4_topic_prefix').value
    if action == 'select-mode':
        command = VehicleCommand(command=VehicleCommand.VEHICLE_CMD_SET_NAV_STATE,
                                 param1=float(status['registered_mode_id']), from_external=True)
        endpoint = 'vehicle_command'
        expected = lambda: status.get('mode') == 'FlyCo'
    else:
        executor = status.get('mode_executor_id')
        if executor is None:
            raise RuntimeError('FlyCo mode executor is not registered')
        # Explicit normal arming only. No force-arm parameter and no auto-arm at startup.
        command = VehicleCommand(command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1., source_component=VehicleCommand.COMPONENT_MODE_EXECUTOR_START+int(executor))
        endpoint = 'vehicle_command_mode_executor'
        expected = lambda: status.get('armed') is True
    publisher = node.create_publisher(VehicleCommand, topic(prefix, 'in', endpoint, VehicleCommand), 1)
    wait_for(node, lambda: publisher.get_subscription_count() > 0, 5., 'PX4 command reader unavailable')
    publisher.publish(command)
    wait_for(node, lambda: time.monotonic()-last_rx[0] < 2. and expected(), 10.,
             action+' not confirmed; inspect PX4/QGC messages (no force or automatic retry)')
    print(action+': confirmed; mode='+str(status.get('mode'))+', armed='+str(status.get('armed')))


def planner_task(node, args):
    joy = node.create_publisher(Joy, '/joy', 10)
    prompt = node.create_publisher(String, '/query_text', 10)
    data = {'odom': None, 'mesh': False}
    odom_topic = '/isaac/odom' if args.sim else '/flyco/odom'
    def odom(msg):
        if msg.header.frame_id == 'world':
            data['odom'] = msg
    def mesh(msg):
        data['mesh'] = msg.vertex_num > 0 and msg.face_num > 0
    node.create_subscription(Odometry, odom_topic, odom, qos_profile_sensor_data)
    node.create_subscription(Mesh, '/prediction/predicted_mesh', mesh, 1)
    publisher = prompt if args.action == 'text' else joy
    def ready():
        msg = data['odom']
        return bool(msg and abs(node.get_clock().now().nanoseconds*1.e-9 -
            (msg.header.stamp.sec+msg.header.stamp.nanosec*1.e-9)) <= .5 and
            publisher.get_subscription_count() > 0 and (args.action != 'scan' or data['mesh']))
    wait_for(node, ready, 30. if args.action == 'scan' else 5.,
             'Need fresh '+odom_topic+', a task subscriber'+(' and predicted mesh' if args.action == 'scan' else ''))
    if args.action == 'text':
        if not args.text or not args.text.strip():
            raise RuntimeError('text requires a nonempty quoted prompt')
        prompt.publish(String(data=args.text.strip()))
    else:
        button = {'initialize': 4, 'enable-planning': 2, 'scan': 5}[args.action]
        message = Joy(buttons=[0]*12, axes=[0.]*8)
        message.buttons[button] = 1
        message.header.stamp = node.get_clock().now().to_msg()
        joy.publish(message)
    deadline = time.monotonic()+.5
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    if args.action != 'text':
        release = Joy(buttons=[0]*12, axes=[0.]*8)
        release.header.stamp = node.get_clock().now().to_msg()
        joy.publish(release)
        rclpy.spin_once(node, timeout_sec=.05)
    print('Sent:', args.action)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', choices=[*CONTROL_ACTIONS, 'status', 'select-mode', 'arm',
                        'initialize', 'enable-planning', 'scan', 'text'])
    parser.add_argument('text', nargs='?')
    parser.add_argument('--sim', action='store_true', help='Use Isaac odometry and /clock')
    args = parser.parse_args(remove_ros_args()[1:])
    if args.action is None:
        from flyco_common.execution import main_for
        main_for(GroundHeartbeat)
        return
    rclpy.init()
    node = Node('flyco_task', automatically_declare_parameters_from_overrides=True,
                parameter_overrides=[Parameter('use_sim_time', value=True)] if args.sim else [])
    try:
        if args.action in CONTROL_ACTIONS:
            control_task(node, args.action)
        elif args.action in ('status', 'select-mode', 'arm'):
            mode_task(node, args.action)
        else:
            planner_task(node, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
