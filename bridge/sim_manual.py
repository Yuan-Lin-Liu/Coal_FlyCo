"""Neutral virtual joystick for Position mode in the isolated native SITL session."""
import time
from px4_msgs.msg import ManualControlSetpoint, VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data
from flyco_real.px4_dds import topic


class NeutralSimSticks:
    def __init__(self, node):
        self.position = self.received_at = None
        self.publisher = node.create_publisher(ManualControlSetpoint,
            topic('/fmu', 'in', 'manual_control_input', ManualControlSetpoint), 1)
        node.create_subscription(VehicleLocalPosition,
            topic('/fmu', 'out', 'vehicle_local_position', VehicleLocalPosition),
            self.on_position, qos_profile_sensor_data)
        self.timer = node.create_timer(.05, self.publish)

    def on_position(self, msg):
        self.position, self.received_at = msg, time.monotonic()

    def publish(self):
        if self.position is None or time.monotonic()-self.received_at > .3:
            return
        # Position accepts centered throttle for normal arming and holds zero
        # climb speed. No arm, mode or takeoff command is generated here.
        self.publisher.publish(ManualControlSetpoint(timestamp=self.position.timestamp,
            timestamp_sample=self.position.timestamp, valid=True,
            data_source=ManualControlSetpoint.SOURCE_MAVLINK_5,
            roll=0., pitch=0., yaw=0., throttle=0., sticks_moving=False))
