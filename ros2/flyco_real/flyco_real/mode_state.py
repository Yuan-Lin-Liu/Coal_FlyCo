"""Observe IDs owned by the C++ ModeBase node; no registration or arming replies."""
import json
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class FlyCoModeState:
    def __init__(self, node):
        self.mode_id = self.executor_id = None
        self.error = 'waiting for C++ FlyCo mode registration'
        self.node = node
        node.create_subscription(String, '/flyco/mode/registration', self.on_registration,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_registration(self, msg):
        try:
            data = json.loads(msg.data)
            mode_id, executor_id = int(data['mode_id']), int(data['executor_id'])
            if not (0 <= mode_id < 31 and executor_id >= 0): return
        except (ValueError, KeyError, TypeError):
            return
        self.mode_id, self.executor_id = mode_id, executor_id
        self.error = ''
        self.node.get_logger().info('C++ FlyCo mode registered: mode=%d, executor=%d' % (mode_id, executor_id))
