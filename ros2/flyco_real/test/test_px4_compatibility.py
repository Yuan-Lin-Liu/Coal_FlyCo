"""Message-format verification must reject schema or DDS endpoint mismatches."""
import threading

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import px4_msgs.msg as messages

from flyco_real.px4_compatibility import (
    INTERFACES, PX4Compatibility, message_fields, message_hash,
)
from flyco_real.px4_dds import topic


def test_message_hash_excludes_constants_and_includes_nested_fields(tmp_path):
    (tmp_path/'Child.msg').write_text('uint32 MESSAGE_VERSION=1\nuint8 value\n')
    (tmp_path/'Parent.msg').write_text('uint64 timestamp # microseconds\nChild[2] items\n')
    assert message_fields(tmp_path, 'Parent') == 'uint64 timestamp\nChild[2] items\nuint8 value\n'
    original = message_hash(tmp_path, 'Parent')
    (tmp_path/'Child.msg').write_text('# comment\nuint32 MESSAGE_VERSION=2\nuint8 value\n')
    assert message_hash(tmp_path, 'Parent') == original
    (tmp_path/'Child.msg').write_text('uint16 value\n')
    assert message_hash(tmp_path, 'Parent') != original


@pytest.mark.parametrize('fault', [None, 'hash', 'endpoint', 'unknown_topic', 'protocol'])
def test_px4_format_checker_over_dds(fault):
    rclpy.init()
    checker = PX4Compatibility()
    fake = Node('fake_px4_format')
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    endpoints = []
    response = fake.create_publisher(messages.MessageFormatResponse,
        '/fmu/out/message_format_response', qos_profile_sensor_data)
    types = {'/fmu/'+direction+'/'+name: type_name for direction, name, type_name in INTERFACES}
    requested = []

    def on_request(msg):
        name = bytes(msg.topic_name).split(b'\0', 1)[0].decode('ascii')
        requested.append(name)
        value = message_hash(checker.directory, types[name])
        if fault == 'hash' and name.endswith('arming_check_reply'):
            value ^= 1
        response.publish(messages.MessageFormatResponse(timestamp=0,
            protocol_version=99 if fault == 'protocol' else msg.protocol_version,
            topic_name=msg.topic_name,
            success=not (fault == 'unknown_topic' and name.endswith('arming_check_reply')),
            message_hash=value))

    fake.create_subscription(messages.MessageFormatRequest, '/fmu/in/message_format_request',
                             on_request, qos_profile_sensor_data)
    for direction, name, type_name in INTERFACES:
        if fault == 'endpoint' and name == 'register_ext_component_request':
            continue
        cls = getattr(messages, type_name)
        wire = topic('/fmu', direction, name, cls)
        if direction == 'in':
            endpoints.append(fake.create_subscription(cls, wire, lambda msg: None, qos_profile_sensor_data))
        else:
            endpoints.append(fake.create_publisher(cls, wire, qos_profile_sensor_data))
    worker = threading.Thread(target=executor.spin)
    worker.start()
    try:
        if fault == 'protocol':
            with pytest.raises(RuntimeError, match='protocol version mismatch'):
                checker.check()
        else:
            assert checker.check() == (fault is None)
            assert set(requested) == set(types)
        # The checker creates no publishers capable of changing flight state.
        published = {name for name, _ in checker.get_publisher_names_and_types_by_node(checker.get_name(), checker.get_namespace())}
        assert published <= {'/rosout', '/parameter_events', '/fmu/in/message_format_request'}
    finally:
        executor.shutdown()
        worker.join(timeout=3.)
        checker.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
