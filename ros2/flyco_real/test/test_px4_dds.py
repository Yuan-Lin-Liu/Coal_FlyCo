"""Native ModeBase registration and planner integration over isolated DDS."""
import json
import math
import os
import re
import signal
import subprocess
import time
from functools import lru_cache
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleCommand, TrajectorySetpoint,
    RegisterExtComponentRequest, RegisterExtComponentReply, ArmingCheckRequest, ArmingCheckReply,
    MessageFormatRequest, MessageFormatResponse)
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String
from std_srvs.srv import Trigger
from flyco_real.px4_dds import topic


@lru_cache(None)
def message_fields(name):
    # Emulate the FMU's message-format service required by the unmodified library.
    directory = Path(get_package_share_directory('px4_msgs')) / 'msg'
    basic = {'bool', 'byte', 'char', 'float32', 'float64', 'int8', 'uint8',
             'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'string', 'wstring'}
    result = ''
    for line in (directory / (name+'.msg')).read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if not line or '=' in line: continue
        kind, field = line.split()[:2]
        result += kind+' '+field+'\n'
        base = kind.split('[', 1)[0]
        if base not in basic: result += message_fields(base)
    return result


def test_native_template_registers_without_planner_gates_and_holds(tmp_path):
    rclpy.init()
    node = Node('native_mode_fmu_fixture')
    statuses, checks, targets, commands = [], [], [], []
    live = dict(armed=False, mode=VehicleStatus.NAVIGATION_STATE_POSCTL, odom=False, command=False)
    local = node.create_publisher(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', qos_profile_sensor_data)
    state = node.create_publisher(VehicleStatus, '/fmu/out/vehicle_status_v1', qos_profile_sensor_data)
    odom = node.create_publisher(Odometry, '/flyco/odom', qos_profile_sensor_data)
    planner = node.create_publisher(PositionCommand, '/planning/pos_cmd', 10)
    request = node.create_publisher(ArmingCheckRequest, '/fmu/out/arming_check_request_v1', qos_profile_sensor_data)
    registration = node.create_publisher(RegisterExtComponentReply, '/fmu/out/register_ext_component_reply_v1', qos_profile_sensor_data)
    format_reply = node.create_publisher(MessageFormatResponse, '/fmu/out/message_format_response', qos_profile_sensor_data)
    node.create_subscription(String, '/flyco/control/status', lambda msg: statuses.append(json.loads(msg.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(ArmingCheckReply, '/fmu/in/arming_check_reply_v1', checks.append, qos_profile_sensor_data)
    node.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', targets.append, qos_profile_sensor_data)
    for endpoint in ('vehicle_command', 'vehicle_command_mode_executor'):
        node.create_subscription(VehicleCommand, '/fmu/in/'+endpoint, commands.append, qos_profile_sensor_data)

    def register(msg):
        assert bytes(msg.name).rstrip(b'\0') == b'FlyCo'
        assert msg.register_mode and msg.register_mode_executor and msg.register_arming_check
        assert not msg.not_user_selectable
        registration.publish(RegisterExtComponentReply(timestamp=node.get_clock().now().nanoseconds//1000,
            request_id=msg.request_id, name=msg.name, success=True, arming_check_id=0, mode_id=26,
            mode_executor_id=1, px4_ros2_api_version=msg.px4_ros2_api_version))

    def format_request(msg):
        name = bytes(msg.topic_name).rstrip(b'\0').decode().split('/')[-1]
        name = re.sub(r'_v\d+$', '', name)
        aliases = {'aux_global_position': 'VehicleGlobalPosition', 'config_control_setpoints': 'VehicleControlMode',
            'config_overrides_request': 'ConfigOverrides', 'vehicle_command_mode_executor': 'VehicleCommand',
            'vehicle_visual_odometry': 'VehicleOdometry'}
        kind = aliases.get(name, ''.join(word.capitalize() for word in name.split('_')))
        value = 0x811c9dc5
        for char in message_fields(kind).encode(): value = ((value ^ char)*0x1000193) & 0xffffffff
        format_reply.publish(MessageFormatResponse(protocol_version=msg.protocol_version,
            topic_name=msg.topic_name, success=True, message_hash=value))

    node.create_subscription(RegisterExtComponentRequest, '/fmu/in/register_ext_component_request_v1', register, qos_profile_sensor_data)
    node.create_subscription(MessageFormatRequest, '/fmu/in/message_format_request', format_request, qos_profile_sensor_data)

    def publish():
        now = node.get_clock().now()
        us = now.nanoseconds//1000
        local.publish(VehicleLocalPosition(timestamp=us, timestamp_sample=us, xy_valid=True, z_valid=True,
            v_xy_valid=True, v_z_valid=True, x=2., y=-1., z=-.5, heading=.2))
        state.publish(VehicleStatus(timestamp=us, nav_state=live['mode'], nav_state_user_intention=live['mode'],
            arming_state=VehicleStatus.ARMING_STATE_ARMED if live['armed'] else VehicleStatus.ARMING_STATE_DISARMED,
            executor_in_charge=1, system_id=1, component_id=1, pre_flight_checks_pass=True))
        request.publish(ArmingCheckRequest(timestamp=us, request_id=7, valid_registrations_mask=1))
        if live['odom']:
            msg = Odometry()
            msg.header.frame_id, msg.child_frame_id = 'world', 'base_link'
            msg.header.stamp = now.to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 3., 4., 1.
            msg.pose.pose.orientation.w = 1.
            odom.publish(msg)
        if live['command']:
            msg = PositionCommand()
            msg.header.frame_id, msg.header.stamp = 'world', now.to_msg()
            msg.position.x, msg.position.y, msg.position.z = 3.1, 4., 1.
            msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            planner.publish(msg)

    node.create_timer(.02, publish)
    log_path = tmp_path/'native-mode.log'
    log = log_path.open('w')
    process = subprocess.Popen(['ros2', 'launch', 'flyco_real', 'mode.launch.py'], cwd='/tmp',
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin(seconds, until=lambda: False):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline and not until():
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        spin(15., lambda: bool(statuses and statuses[-1]['mode_registered'] and checks))
        assert statuses and statuses[-1]['mode_registered'], log_path.read_text()
        assert not statuses[-1]['enabled'] and not statuses[-1]['odom_ok'] and not statuses[-1]['ground_ok']
        assert checks[-1].can_arm_and_run
        assert checks[-1].mode_req_local_position_relaxed and not checks[-1].mode_req_local_position
        assert not targets and not commands
        # No planner, ROS odometry or ground heartbeat is needed for native activation/hold.
        live.update(armed=True, mode=26)
        spin(2., lambda: len(targets) >= 5)
        assert targets
        np.testing.assert_allclose(targets[-1].position, [2., -1., -.5])
        assert np.isnan(targets[-1].velocity).all()
        assert checks[-1].can_arm_and_run
        # Planner alignment can start while already in FlyCo, without a Position switch or heartbeat.
        live['odom'] = True
        spin(2., lambda: statuses[-1]['enabled'])
        assert statuses[-1]['enabled'] and not statuses[-1]['ground_ok']
        client = node.create_client(Trigger, '/flyco/control/resume')
        assert client.wait_for_service(timeout_sec=2.)
        response = client.call_async(Trigger.Request())
        spin(2., response.done)
        assert response.result().success, response.result().message
        node.destroy_client(client)
        live['command'] = True
        spin(.5)
        np.testing.assert_allclose(targets[-1].position, [2.+.1*math.cos(.2), -1.+.1*math.sin(.2), -.5], atol=1.e-5)
        assert {p.node_name for p in node.get_publishers_info_by_topic('/fmu/in/arming_check_reply_v1')} == {'flyco_mode'}
        assert {p.node_name for p in node.get_publishers_info_by_topic('/fmu/in/trajectory_setpoint')} == {'flyco_mode'}
        # Loss of planner data falls back to native hold; it does not veto native arming checks.
        live['odom'] = False
        spin(.8)
        assert not statuses[-1]['enabled']
        np.testing.assert_allclose(targets[-1].position, [2., -1., -.5])
        assert checks[-1].can_arm_and_run
        live.update(armed=False, mode=VehicleStatus.NAVIGATION_STATE_POSCTL)
        spin(.2)
        before = len(targets)
        spin(.2)
        assert len(targets) == before
        assert not commands  # No automatic arm or mode-switch request.
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close()
        node.destroy_node(); rclpy.shutdown()
