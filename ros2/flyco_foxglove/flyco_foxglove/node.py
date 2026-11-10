"""Export saved RViz settings and refresh the Foxglove layout on demand."""
import json
from copy import deepcopy
from pathlib import Path
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from .converter import export_layout


class RvizToFoxglove(Node):
    def __init__(self):
        super().__init__('rviz_to_foxglove')
        self.declare_parameter('rviz_config', '')
        self.declare_parameter('output_path', 'runtime/foxglove/flyco_odin.json')
        self.source = Path(self.get_parameter('rviz_config').value).expanduser().resolve()
        self.output = Path(self.get_parameter('output_path').value).expanduser().resolve()
        self._source_signature = None
        self._pose_adapters = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '~/layout', qos)
        self.reload_service = self.create_service(Trigger, '~/reload', self.reload_callback)
        if not self.refresh():
            raise ValueError('Could not export the initial RViz configuration')
        self.watch_timer = self.create_timer(1., self.watch)

    def refresh(self):
        try:
            layout, report = export_layout(self.source, self.output)
            self.configure_pose_adapters(report['odometry_adapters'])
            stat = self.source.stat()
            self._source_signature = (stat.st_mtime_ns, stat.st_size)
            self.publisher.publish(String(data=json.dumps(layout, ensure_ascii=False)))
            self.get_logger().info(
                f'Foxglove layout ready: {self.output}; {len(report["topics"])} topics. '
                'Import this JSON with Layouts > Import from file.')
            for entry in report['unsupported']:
                self.get_logger().warning(f'Unsupported RViz display: {entry["name"]} ({entry["class"]})')
            return True
        except (OSError, ValueError, TypeError) as error:
            self.get_logger().error(f'Layout export failed; last valid layout retained: {error}',
                                    throttle_duration_sec=5.)
            return False

    def publish_pose(self, message, publisher):
        pose = PoseStamped()
        pose.header = deepcopy(message.header)
        pose.pose = deepcopy(message.pose.pose)
        publisher.publish(pose)

    def configure_pose_adapters(self, mappings):
        wanted = {(entry['source'], entry['target']) for entry in mappings}
        for key in self._pose_adapters.keys() - wanted:
            subscription, publisher = self._pose_adapters.pop(key)
            self.destroy_subscription(subscription)
            self.destroy_publisher(publisher)
        for source, target in wanted - self._pose_adapters.keys():
            publisher = self.create_publisher(PoseStamped, target, 5)
            subscription = self.create_subscription(
                Odometry, source,
                lambda message, output=publisher: self.publish_pose(message, output),
                qos_profile_sensor_data)
            self._pose_adapters[(source, target)] = (subscription, publisher)

    def watch(self):
        try:
            stat = self.source.stat()
            changed = (stat.st_mtime_ns, stat.st_size) != self._source_signature
        except OSError:
            changed = True
        if changed:
            self.refresh()

    def reload_callback(self, request, response):
        response.success = self.refresh()
        response.message = str(self.output) if response.success else 'Export failed; see node log'
        return response


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    previous = {number: signal.signal(number, lambda *_: stop.set())
                for number in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = RvizToFoxglove()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for number, handler in previous.items():
            signal.signal(number, handler)
