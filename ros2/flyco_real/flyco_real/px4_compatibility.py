"""Read-only verification of FlyCo DDS endpoints and PX4 message format hashes."""
import math
from pathlib import Path
import re
import time

from ament_index_python.packages import get_package_share_directory
import px4_msgs.msg as messages
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .px4_dds import topic


# The names here are DDS/uORB aliases, which can differ from the message type.
INTERFACES = (
    ('in', 'register_ext_component_request', 'RegisterExtComponentRequest'),
    ('out', 'register_ext_component_reply', 'RegisterExtComponentReply'),
    ('out', 'arming_check_request', 'ArmingCheckRequest'),
    ('in', 'arming_check_reply', 'ArmingCheckReply'),
    ('in', 'unregister_ext_component', 'UnregisterExtComponent'),
    ('in', 'config_control_setpoints', 'VehicleControlMode'),
    ('in', 'trajectory_setpoint', 'TrajectorySetpoint'),
    ('in', 'vehicle_command', 'VehicleCommand'),
    ('in', 'vehicle_command_mode_executor', 'VehicleCommand'),
    ('in', 'vehicle_visual_odometry', 'VehicleOdometry'),
    ('out', 'vehicle_status', 'VehicleStatus'),
    ('out', 'vehicle_local_position', 'VehicleLocalPosition'),
    ('out', 'vehicle_land_detected', 'VehicleLandDetected'),
    ('out', 'vehicle_command_ack', 'VehicleCommandAck'),
    ('out', 'mode_completed', 'ModeCompleted'),
)
_BASIC_TYPES = frozenset(('bool', 'byte', 'char', 'float32', 'float64',
                         'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32',
                         'int64', 'uint64', 'string', 'wstring'))
_FIELD = re.compile(r'([A-Za-z0-9_/]+)(\[[0-9]*\])?\s+(\w+)')


def message_fields(directory, name, ancestors=()):
    """Canonical fields used by PX4's MessageFormatResponse FNV-1a hash.

    Constants/comments are excluded. Nested definitions follow their field,
    including for arrays, as in Tools/msg/px_generate_uorb_topic_helper.py.
    """
    if name in ancestors:
        raise ValueError('Recursive PX4 message: '+name)
    result = ''
    for raw in (Path(directory)/(name+'.msg')).read_text().splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line or '=' in line:
            continue
        match = _FIELD.fullmatch(line)
        if match is None:
            raise ValueError('Unsupported field in %s: %s' % (name, line))
        base, array, field = match.groups()
        base = base.rsplit('/', 1)[-1]
        result += base+(array or '')+' '+field+'\n'
        if base not in _BASIC_TYPES:
            result += message_fields(directory, base, ancestors+(name,))
    return result


def message_hash(directory, name):
    value = 0x811c9dc5
    for byte in message_fields(directory, name).encode('ascii'):
        value = ((value ^ byte)*0x1000193) & 0xffffffff
    return value


class PX4Compatibility(Node):
    def __init__(self):
        super().__init__('check_px4', automatically_declare_parameters_from_overrides=True)
        for key, default in (('px4_topic_prefix', '/fmu'), ('timeout', 3.), ('discovery_timeout', 10.)):
            if not self.has_parameter(key):
                self.declare_parameter(key, default)
        self.prefix = self.get_parameter('px4_topic_prefix').value
        self.timeout = float(self.get_parameter('timeout').value)
        self.discovery_timeout = float(self.get_parameter('discovery_timeout').value)
        if any(not math.isfinite(v) or v <= 0. for v in (self.timeout, self.discovery_timeout)):
            raise ValueError('Timeouts must be positive and finite')
        self.directory = Path(get_package_share_directory('px4_msgs'))/'msg'
        self.responses = {}
        self.request = self.create_publisher(messages.MessageFormatRequest,
            topic(self.prefix, 'in', 'message_format_request', messages.MessageFormatRequest), 1)
        self.response = self.create_subscription(messages.MessageFormatResponse,
            topic(self.prefix, 'out', 'message_format_response', messages.MessageFormatResponse),
            self.on_response, qos_profile_sensor_data)

    def on_response(self, msg):
        name = bytes(msg.topic_name).split(b'\0', 1)[0].decode('ascii')
        self.responses[name] = msg

    def query(self, base_topic):
        # PX4 looks up the uORB alias, without the DDS _vN suffix.
        encoded = base_topic.encode('ascii')
        if len(encoded) >= 50:
            raise ValueError('PX4 format request topic is too long: '+base_topic)
        msg = messages.MessageFormatRequest(timestamp=0,
            protocol_version=messages.MessageFormatRequest.LATEST_PROTOCOL_VERSION,
            topic_name=list(encoded+bytes(50-len(encoded))))
        self.responses.pop(base_topic, None)
        deadline, next_send = time.monotonic()+self.timeout, 0.
        while rclpy.ok() and time.monotonic() < deadline:
            if time.monotonic() >= next_send:
                self.request.publish(msg)
                next_send = time.monotonic()+.3
            rclpy.spin_once(self, timeout_sec=.05)
            if base_topic in self.responses:
                response = self.responses.pop(base_topic)
                if response.protocol_version != msg.protocol_version:
                    raise RuntimeError('PX4 message format protocol version mismatch')
                return response
        raise TimeoutError('No PX4 message format reply for '+base_topic)

    def check(self):
        deadline = time.monotonic()+self.discovery_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.request.get_subscription_count() and self.count_publishers(self.response.topic_name):
                break
            rclpy.spin_once(self, timeout_sec=.1)
        else:
            raise TimeoutError('PX4 format endpoints not found; check Agent, DDS domain and firmware')
        passed = True
        for direction, name, type_name in INTERFACES:
            expected = message_hash(self.directory, type_name)
            wire_topic = topic(self.prefix, direction, name, getattr(messages, type_name))
            base_topic = '/'+self.prefix.strip('/')+'/'+direction+'/'+name
            response = self.query(base_topic)
            endpoints = (self.get_subscriptions_info_by_topic(wire_topic) if direction == 'in'
                         else self.get_publishers_info_by_topic(wire_topic))
            if not response.success:
                self.get_logger().error('PX4 has no uORB definition for '+base_topic)
                passed = False
            elif response.message_hash != expected:
                self.get_logger().error('Message mismatch: %s (local=%08x, PX4=%08x)' %
                    (wire_topic, expected, response.message_hash))
                passed = False
            elif not endpoints:
                self.get_logger().error('Message format matches but DDS endpoint is missing: '+wire_topic)
                passed = False
            else:
                self.get_logger().info('PASS '+wire_topic)
        if passed:
            self.get_logger().info('PX4 message check passed (%d interfaces)' % len(INTERFACES))
        return passed


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PX4Compatibility()
        return 0 if node.check() else 1
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError) as error:
        print('PX4 message check failed: '+str(error), flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
