"""PX4 external-mode registration, arming checks and controller requirements.

PX4 publishes this registration to QGC through MAVLink AVAILABLE_MODES.
The Offboard mode is neither renamed nor replaced.
"""
import secrets
import time

from px4_msgs.msg import (RegisterExtComponentRequest, RegisterExtComponentReply,
    ArmingCheckRequest, ArmingCheckReply, VehicleControlMode, UnregisterExtComponent)
from rclpy.qos import qos_profile_sensor_data
from .px4_dds import topic


class FlyCoMode:
    def __init__(self, node, prefix, healthy):
        self.node, self.healthy = node, healthy
        self.mode_id = self.arming_check_id = self.executor_id = None
        self.request_id = secrets.randbits(63)
        self.last_request = 0.
        self.attempts = 0
        self.error = ''
        self.register = node.create_publisher(RegisterExtComponentRequest,
            topic(prefix, 'in', 'register_ext_component_request', RegisterExtComponentRequest), 1)
        self.unregister = node.create_publisher(UnregisterExtComponent,
            topic(prefix, 'in', 'unregister_ext_component', UnregisterExtComponent), 1)
        self.check_reply = node.create_publisher(ArmingCheckReply,
            topic(prefix, 'in', 'arming_check_reply', ArmingCheckReply), 1)
        self.config = node.create_publisher(VehicleControlMode,
            topic(prefix, 'in', 'config_control_setpoints', VehicleControlMode), 1)
        node.create_subscription(RegisterExtComponentReply,
            topic(prefix, 'out', 'register_ext_component_reply', RegisterExtComponentReply),
            self.on_registration, qos_profile_sensor_data)
        node.create_subscription(ArmingCheckRequest,
            topic(prefix, 'out', 'arming_check_request', ArmingCheckRequest),
            self.on_check, qos_profile_sensor_data)

    def tick(self):
        if self.mode_id is not None:
            msg = VehicleControlMode(timestamp=self.node.timestamp_us(), source_id=self.mode_id)
            for field in ('flag_control_auto_enabled', 'flag_control_position_enabled', 'flag_control_velocity_enabled',
                          'flag_control_altitude_enabled', 'flag_control_climb_rate_enabled',
                          'flag_control_acceleration_enabled', 'flag_control_attitude_enabled',
                          'flag_control_rates_enabled', 'flag_control_allocation_enabled'):
                setattr(msg, field, True)
            self.config.publish(msg)
            return
        now = time.monotonic()
        if self.register.get_subscription_count() == 0:
            self.error = 'waiting for PX4 registration reader; check DDS domain and message version'
            return
        if self.attempts >= 3 or now-self.last_request < 3.:
            return
        msg = RegisterExtComponentRequest(timestamp=self.node.timestamp_us(),
            request_id=self.request_id,
            px4_ros2_api_version=RegisterExtComponentRequest.LATEST_PX4_ROS2_API_VERSION,
            register_arming_check=True, register_mode=True, register_mode_executor=True,
            not_user_selectable=False)
        msg.name = list(b'FlyCo'+bytes(20))
        self.register.publish(msg)
        self.last_request = now
        self.attempts += 1
        self.error = 'waiting for PX4 external-mode registration'

    def on_registration(self, msg):
        if msg.request_id != self.request_id:
            return
        if not msg.success or msg.mode_id < 0 or msg.arming_check_id < 0 or msg.mode_executor_id < 0:
            self.error = 'PX4 rejected FlyCo mode registration'
            return
        if msg.px4_ros2_api_version != RegisterExtComponentRequest.LATEST_PX4_ROS2_API_VERSION:
            self.error = 'PX4 ROS2 mode API version mismatch'
            return
        self.mode_id, self.arming_check_id = int(msg.mode_id), int(msg.arming_check_id)
        self.executor_id = int(msg.mode_executor_id)
        self.error = ''
        self.node.get_logger().info('Registered FlyCo: nav_state=%d, arming_check=%d' %
                                    (self.mode_id, self.arming_check_id))

    def on_check(self, msg):
        if self.arming_check_id is None:
            return
        reply = ArmingCheckReply(timestamp=self.node.timestamp_us(),
            request_id=msg.request_id, registration_id=self.arming_check_id,
            can_arm_and_run=bool(self.healthy()), mode_req_angular_velocity=True,
            mode_req_attitude=True, mode_req_local_alt=True, mode_req_local_position=True)
        self.check_reply.publish(reply)

    def close(self):
        if self.mode_id is None:
            return
        msg = UnregisterExtComponent(timestamp=self.node.timestamp_us(),
            arming_check_id=self.arming_check_id, mode_id=self.mode_id, mode_executor_id=self.executor_id)
        msg.name = list(b'FlyCo'+bytes(20))
        self.unregister.publish(msg)
