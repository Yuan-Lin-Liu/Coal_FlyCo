"""Initialize FlyCo and take off once after an armed, landed Position -> FlyCo switch."""
import json
import math
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FlyCoMission(Node):
    def __init__(self):
        super().__init__('flyco_mission', automatically_declare_parameters_from_overrides=True)
        def param(name, default):
            if not self.has_parameter(name): self.declare_parameter(name, default)
            return self.get_parameter(name).value
        self.odom_topic = param('odom_topic', '/flyco/odom')
        self.gimbal_topic = param('gimbal_topic', '/flyco/gimbal_pose')
        self.odom = self.odom_rx = self.gimbal_rx = None
        self.control, self.control_rx = {}, None
        self.initialized = False
        self.last_initial = -math.inf
        self.release_at = None
        self.activation_deadline = None
        self.takeoff_future = self.request_sent_at = None
        self.target_z = None
        self.phase = 'initializing'
        self._last_status = None
        self.joy = self.create_publisher(Joy, '/joy', 10)
        self.status = self.create_publisher(String, '/flyco/mission/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.takeoff = self.create_client(Trigger, '/flyco/control/takeoff')
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(QuaternionStamped, self.gimbal_topic, self.on_gimbal, qos_profile_sensor_data)
        self.create_subscription(String, '/flyco/control/status', self.on_control,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self.tick)
        self.report('initializing', 'Waiting for FlyCo odometry, camera pose and planner')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if msg.header.frame_id == 'world' and all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.odom, self.odom_rx = msg, time.monotonic()

    def on_gimbal(self, msg):
        q = msg.quaternion
        if all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            self.gimbal_rx = time.monotonic()

    def on_control(self, msg):
        previous_mode = self.control.get('mode')
        self.control, self.control_rx = json.loads(msg.data), time.monotonic()
        if previous_mode == 'POSCTL' and self.control.get('mode') == 'FlyCo':
            if not self.initialized:
                self.report('blocked', 'Planner not initialized; return to Position before trying again')
            elif not self.control.get('armed') or self.control.get('landed') is not True:
                self.report('waiting_for_position', 'Auto takeoff requires an already armed vehicle on the ground')
            elif self.takeoff_future is None:
                self.activation_deadline = time.monotonic()+3.
                self.report('activating', 'Position -> FlyCo detected; waiting for mode executor ownership')

    def odom_ok(self):
        if self.odom is None or self.odom_rx is None or time.monotonic()-self.odom_rx > .4:
            return False
        stamp = self.odom.header.stamp
        return abs(self.get_clock().now().nanoseconds*1.e-9-(stamp.sec+stamp.nanosec*1.e-9)) <= .4

    def publish_joy(self, button=None):
        msg = Joy(buttons=[0]*12, axes=[0.]*8)
        msg.header.stamp = self.get_clock().now().to_msg()
        if button is not None: msg.buttons[button] = 1
        self.joy.publish(msg)

    def tick(self):
        now = time.monotonic()
        if self.release_at is not None and now >= self.release_at:
            self.publish_joy()
            self.release_at = None
        if not self.initialized:
            if self.count_publishers('/local_newest_traj') > 0:
                self.initialized = True
                self.report('waiting_for_position', 'FlyCo ready; normally arm on the ground in Position, then select FlyCo')
            elif (self.odom_ok() and self.gimbal_rx is not None and now-self.gimbal_rx <= .4 and
                    self.joy.get_subscription_count() >= 1 and now-self.last_initial >= 1.):
                self.publish_joy(4)
                self.last_initial, self.release_at = now, now+.2
            return
        if self.activation_deadline is not None:
            if self.control.get('mode') != 'FlyCo' or now > self.activation_deadline:
                self.activation_deadline = None
                self.report('blocked', 'Activation cancelled or timed out; return to Position to try again')
            elif (self.odom_ok() and self.control_rx is not None and now-self.control_rx <= 2. and
                    all(self.control.get(k) for k in ('enabled','armed','odom_ok','px4_ok','mode_executor_active')) and
                    self.control.get('landed') is True and self.takeoff.service_is_ready()):
                self.target_z = self.odom.pose.pose.position.z+float(self.control.get('takeoff_height', 2.))
                self.activation_deadline = None
                self.takeoff_future = self.takeoff.call_async(Trigger.Request())
                self.request_sent_at = now
                self.report('requesting_takeoff', 'Requesting PX4 native Takeoff once')
        if self.takeoff_future is not None:
            if self.takeoff_future.done():
                try:
                    result = self.takeoff_future.result()
                    if result is not None and result.success:
                        self.report('taking_off', result.message)
                    else:
                        self.report('blocked', result.message if result else 'No takeoff response')
                except Exception as error:
                    self.report('blocked', str(error))
                self.takeoff_future = None
            elif now-self.request_sent_at > 5.:
                self.takeoff_future.cancel()
                self.takeoff_future = None
                self.report('blocked', 'Takeoff response timed out; inspect PX4 mode before another activation')
        if self.phase == 'taking_off':
            if self.control.get('takeoff_phase') == 'failed':
                self.report('blocked', self.control.get('reason', 'Takeoff interrupted'))
            elif (self.control.get('takeoff_phase') == 'complete' and self.control.get('mode') == 'FlyCo' and
                    self.odom_ok() and self.target_z is not None and
                    abs(self.odom.pose.pose.position.z-self.target_z) < .15):
                self.report('hovering', 'At takeoff height; holding in FlyCo, ready for target text and scanning')

    def report(self, phase, message):
        self.phase = phase
        value = json.dumps(dict(phase=phase, initialized=self.initialized, target_z=self.target_z, message=message))
        if value != self._last_status:
            self.status.publish(String(data=value))
            self.get_logger().info(value)
            self._last_status = value


def main():
    from flyco_common.execution import main_for
    main_for(FlyCoMission)
