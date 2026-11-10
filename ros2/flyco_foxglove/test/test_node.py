import json
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from flyco_foxglove.node import RvizToFoxglove
from nav_msgs.msg import Odometry


def exporter(tmp_path):
    node = object.__new__(RvizToFoxglove)
    node.source = tmp_path / 'source.rviz'
    node.output = tmp_path / 'out/layout.json'
    node._source_signature = None
    node._pose_adapters = {}
    node.source.write_text('Visualization Manager:\n  Global Options:\n    Fixed Frame: world\n  Displays: []\n')
    messages, errors = [], []
    node.publisher = SimpleNamespace(publish=messages.append)
    node.get_logger = lambda: SimpleNamespace(
        info=lambda message: None, warning=lambda message: None,
        error=lambda message, **kwargs: errors.append(message))
    return node, messages, errors


def test_exporter_publishes_layout_and_reloads_saved_frame_changes(tmp_path):
    node, messages, errors = exporter(tmp_path)
    assert node.refresh()
    node.source.write_text(node.source.read_text().replace('world', 'odom'))
    node.watch()

    assert len(messages) == 2
    assert json.loads(messages[-1].data)['configById']['3D!flyco']['fixedFrame'] == 'odom'
    assert not errors
    node.watch()
    assert len(messages) == 2


def test_reload_service_reports_failure_and_keeps_previous_layout(tmp_path):
    node, messages, errors = exporter(tmp_path)
    assert node.refresh()
    previous = node.output.read_bytes()
    node.source.write_text('[invalid yaml')
    response = node.reload_callback(None, SimpleNamespace(success=True, message=''))

    assert not response.success
    assert node.output.read_bytes() == previous
    assert len(messages) == 1
    assert errors


def test_odometry_adapter_preserves_pose_time_and_frame_without_mutating_source(tmp_path):
    node, _, _ = exporter(tmp_path)
    output = []
    message = Odometry()
    message.header.frame_id = 'world'
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.pose.pose.position.x = 2.5
    message.pose.pose.orientation.w = 1.
    before = repr(message)

    node.publish_pose(message, SimpleNamespace(publish=output.append))

    assert len(output) == 1
    assert output[0].header == message.header
    assert output[0].pose == message.pose.pose
    assert repr(message) == before
