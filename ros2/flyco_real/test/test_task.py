"""Exercise terminal tasks over ROS2, including normal arming and failure reporting."""
import json
import threading
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from flyco_real.px4_dds import topic
from flyco_real.task import CONTROL_ACTIONS, control_task, mode_task, planner_task


@pytest.fixture
def terminal():
    rclpy.init()
    server = Node('task_test_server')
    client = Node('task_test_client', parameter_overrides=[Parameter('px4_topic_prefix', value='/task_test/fmu')],
                  automatically_declare_parameters_from_overrides=True)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    worker = threading.Thread(target=executor.spin)
    worker.start()
    try:
        yield server, client
    finally:
        executor.shutdown()
        worker.join(timeout=3.)
        client.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_control_tasks_report_rejections_and_land_without_odometry(terminal):
    server, client = terminal
    received = []
    def accept(name):
        def callback(request, response):
            received.append((name, getattr(request, 'data', None)))
            response.success, response.message = True, 'accepted'
            return response
        return callback
    # Enable/disable share one endpoint and type.
    for name, kind in {name: kind for name, kind, _ in CONTROL_ACTIONS.values()}.items():
        server.create_service(kind, '/flyco/control/'+name, accept(name))
    for action in CONTROL_ACTIONS:
        control_task(client, action)
    assert ('enable', True) in received and ('enable', False) in received
    assert ('land', None) in received and received.count(('takeoff', None)) == 2
    assert all(name != 'arm' for name, _ in received)


def test_control_task_does_not_hide_rejection(terminal):
    server, client = terminal
    def reject(request, response):
        response.success, response.message = False, 'position unavailable'
        return response
    server.create_service(Trigger, '/flyco/control/hold', reject)
    with pytest.raises(RuntimeError, match='position unavailable'):
        control_task(client, 'hold')


def test_terminal_mode_selection_and_normal_arm_confirm_actual_state(terminal):
    server, client = terminal
    state = dict(registered_mode_id=26, mode_executor_id=2, enabled=True, px4_ok=True,
                 odom_ok=True, ground_ok=True, armed=False, mode='POSCTL')
    received = []
    status = server.create_publisher(String, '/flyco/control/status',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def command(msg):
        received.append(msg)
        if msg.command == VehicleCommand.VEHICLE_CMD_SET_NAV_STATE:
            state['mode'] = 'FlyCo'
        elif msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            state['armed'] = True
    for endpoint in ('vehicle_command', 'vehicle_command_mode_executor'):
        server.create_subscription(VehicleCommand, topic('/task_test/fmu', 'in', endpoint, VehicleCommand),
                                   command, qos_profile_sensor_data)
    server.create_timer(.05, lambda: status.publish(String(data=json.dumps(state))))
    with pytest.raises(RuntimeError, match='Select FlyCo before arming'):
        mode_task(client, 'arm')
    mode_task(client, 'select-mode')
    mode_task(client, 'arm')
    assert received[0].param1 == 26.
    assert received[1].source_component == VehicleCommand.COMPONENT_MODE_EXECUTOR_START+2
    assert received[1].param1 == 1. and received[1].param2 == 0.
    with pytest.raises(RuntimeError, match='Already armed'):
        mode_task(client, 'arm')
    assert len(received) == 2


def test_sim_task_uses_isaac_odometry_and_releases_joy(terminal):
    server, client = terminal
    client.set_parameters([Parameter('use_sim_time', value=True)])
    clock = server.create_publisher(Clock, '/clock', 1)
    odom = server.create_publisher(Odometry, '/isaac/odom', qos_profile_sensor_data)
    received, prompts = [], []
    server.create_subscription(Joy, '/joy', received.append, 10)
    server.create_subscription(String, '/query_text', prompts.append, 10)
    def publish():
        message = Odometry()
        message.header.frame_id = 'world'
        message.header.stamp.sec = 10
        message.pose.pose.orientation.w = 1.
        odom.publish(message)
        tick = Clock()
        tick.clock.sec = 10
        clock.publish(tick)
    server.create_timer(.02, publish)
    planner_task(client, SimpleNamespace(action='initialize', sim=True, text=None))
    assert any(msg.buttons[4] == 1 for msg in received)
    assert list(received[-1].buttons) == [0]*12
    planner_task(client, SimpleNamespace(action='text', sim=True, text='一个煤堆'))
    assert prompts[-1].data == '一个煤堆'
