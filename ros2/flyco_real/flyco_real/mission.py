"""Coordinate takeoff, target text, planning, tracking and one complete scan."""
import json
import math
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger
from quadrotor_msgs.msg import Mesh, PolynomialTrajGroup


class FlyCoMission(Node):
    def __init__(self, **kwargs):
        super().__init__('flyco_mission', automatically_declare_parameters_from_overrides=True, **kwargs)
        def param(name, default):
            if not self.has_parameter(name): self.declare_parameter(name, default)
            return self.get_parameter(name).value
        self.odom_topic = param('odom_topic', '/flyco/odom')
        self.gimbal_topic = param('gimbal_topic', '/flyco/gimbal_pose')
        self.takeoff_on_armed_flyco = param('takeoff_on_armed_flyco', False)
        self.auto_scan = param('auto_scan', False)
        self.text = ''
        self.prompt_stamp = self.cloud_stamp = None
        self.mesh_ready = False
        self.coverage = None
        self.plan_stamp = self.scan_stamp = None
        self.scan_started = None
        self.trajectory_seen = self.scan_finished = self.resume_sent = False
        self.scan_future = self.scan_action = self.scan_request_at = None
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
        self.resume = self.create_client(Trigger, '/flyco/control/resume')
        self.hold = self.create_client(Trigger, '/flyco/control/hold')
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_subscription(QuaternionStamped, self.gimbal_topic, self.on_gimbal, qos_profile_sensor_data)
        self.create_subscription(String, '/flyco/control/status', self.on_control,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.auto_scan:
            self.create_subscription(String, '/query_text', self.on_text, 10)
            self.create_subscription(PointCloud2, '/tracking/target_pc', self.on_target_cloud, qos_profile_sensor_data)
            self.create_subscription(Mesh, '/prediction/predicted_mesh', self.on_mesh, 1)
            self.create_subscription(PolynomialTrajGroup, '/local_newest_traj', self.on_trajectory, 10)
            self.create_subscription(Bool, '/flight_end', self.on_scan_finished, 10)
            self.create_subscription(Float32, '/planning/flight_coverage', self.on_coverage, 1)
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
        previously_armed = self.control.get('armed', False)
        self.control, self.control_rx = json.loads(msg.data), time.monotonic()
        if self.activation_deadline is not None and (self.control.get('mode') != 'FlyCo' or
                not self.control.get('armed') or self.control.get('landed') is not True):
            self.activation_deadline = None
            self.report('waiting_for_arming', 'Activation cancelled; select FlyCo and normally arm to try again')
        activated = previous_mode == 'POSCTL' and self.control.get('mode') == 'FlyCo'
        if self.takeoff_on_armed_flyco:
            activated = (self.control.get('mode') == 'FlyCo' and self.control.get('armed') is True and
                         self.control.get('landed') is True and
                         (previous_mode != 'FlyCo' or not previously_armed))
        if self.auto_scan and self.scan_stamp is not None:
            activated = False
        if activated:
            if not self.initialized and not self.takeoff_on_armed_flyco:
                self.report('blocked', 'Planner not initialized; return to Position before trying again')
            elif not self.control.get('armed') or self.control.get('landed') is not True:
                self.report('waiting_for_position', 'Auto takeoff requires an already armed vehicle on the ground')
            elif self.takeoff_future is None:
                self.activation_deadline = time.monotonic()+(3. if self.initialized else 30.)
                self.report('activating', 'Armed FlyCo ground activation; waiting for initialization and mode executor ownership')

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

    def on_text(self, msg):
        text = msg.data.strip()
        if not text or text == self.text:
            return
        if self.phase in ('enabling_planning', 'planning', 'scanning', 'finishing', 'completed'):
            if self.hold.service_is_ready():
                self.hold.call_async(Trigger.Request())
            self.report('blocked', 'Target changed during the scan; holding position. Restart the mission for a new target')
            return
        self.text = text
        self.prompt_stamp = self.get_clock().now().nanoseconds*1.e-9
        self.cloud_stamp = None
        self.mesh_ready = False

    def on_target_cloud(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1.e-9
        if self.text and msg.width*msg.height > 0 and stamp > self.prompt_stamp:
            self.cloud_stamp = stamp

    def on_mesh(self, msg):
        if self.cloud_stamp is not None and msg.vertex_num > 0 and msg.face_num > 0:
            self.mesh_ready = True

    def on_trajectory(self, msg):
        stamp = msg.start_time.sec+msg.start_time.nanosec*1.e-9
        if (self.scan_stamp is not None and stamp >= self.scan_stamp-.1 and
                msg.action == PolynomialTrajGroup.ACTION_ADD and msg.pos_traj):
            self.trajectory_seen = True

    def on_scan_finished(self, msg):
        if self.phase in ('planning', 'scanning'):
            self.scan_finished = bool(msg.data)

    def on_coverage(self, msg):
        if math.isfinite(msg.data):
            self.coverage = float(msg.data)

    def scan_request(self, action):
        client = self.resume if action == 'resume' else self.hold
        if not client.service_is_ready():
            self.report('blocked', 'FlyCo '+action+' service unavailable')
            return
        self.scan_action, self.scan_request_at = action, time.monotonic()
        self.scan_future = client.call_async(Trigger.Request())

    def tick_scan(self, now):
        if self.phase not in ('hovering', 'waiting_for_text', 'waiting_for_mesh',
                              'enabling_planning', 'planning', 'scanning', 'finishing'):
            return
        if not (self.control.get('mode') == 'FlyCo' and self.control.get('armed') and
                all(self.control.get(k) for k in ('enabled', 'odom_ok', 'px4_ok', 'ground_ok')) and
                self.control_rx is not None and now-self.control_rx <= 2. and self.odom_ok()):
            if self.hold.service_is_ready():
                self.hold.call_async(Trigger.Request())
            self.report('blocked', 'Automatic scan stopped: FlyCo mode or localization/control readiness was lost')
            return
        if self.scan_future is not None:
            if self.scan_future.done():
                try:
                    result = self.scan_future.result()
                    if result is None or not result.success:
                        self.report('blocked', result.message if result else 'No scan control response')
                    elif self.scan_action == 'hold':
                        self.report('completed', 'Planner reported scan completion; holding the current position')
                except Exception as error:
                    self.report('blocked', str(error))
                self.scan_future = None
                return
            if now-self.scan_request_at > 5.:
                self.scan_future.cancel()
                self.scan_future = None
                self.report('blocked', 'Scan control response timed out')
            return
        if self.phase in ('hovering', 'waiting_for_text', 'waiting_for_mesh'):
            if not self.text:
                self.report('waiting_for_text', 'Hovering; enter the target description in FlyCo-UI and send it')
            elif not self.mesh_ready:
                self.report('waiting_for_mesh', 'Target received; waiting for its point cloud and predicted mesh')
            elif self.joy.get_subscription_count() > 0:
                self.publish_joy(2)
                self.plan_stamp = self.get_clock().now().nanoseconds*1.e-9
                self.report('enabling_planning', 'Target mesh ready; enabling FlyCo planning')
        elif self.phase == 'enabling_planning':
            if self.get_clock().now().nanoseconds*1.e-9-self.plan_stamp >= .5 and self.release_at is None:
                self.scan_stamp = self.get_clock().now().nanoseconds*1.e-9
                self.scan_started = now
                self.publish_joy(5)
                self.report('planning', 'Planning and scan triggered; waiting for the first flight trajectory')
        elif self.phase in ('planning', 'scanning'):
            if self.scan_finished:
                if not self.trajectory_seen:
                    self.report('blocked', 'Planner ended without a scanning trajectory; inspect target geometry')
                else:
                    self.report('finishing', 'Scan finished; stopping trajectory tracking')
                    self.scan_request('hold')
            elif self.phase == 'planning':
                if self.trajectory_seen and self.control.get('tracking'):
                    self.report('scanning', 'Following FlyCo scan trajectories')
                elif self.trajectory_seen and not self.resume_sent:
                    self.resume_sent = True
                    self.scan_request('resume')
                elif now-self.scan_started > 120.:
                    if self.hold.service_is_ready():
                        self.hold.call_async(Trigger.Request())
                    self.report('blocked', 'No scanning trajectory started within 120 seconds')

    def tick(self):
        now = time.monotonic()
        if self.release_at is not None and now >= self.release_at:
            self.publish_joy()
            self.release_at = None
        if not self.initialized:
            if self.count_publishers('/local_newest_traj') > 0:
                self.initialized = True
                if self.takeoff_on_armed_flyco:
                    self.report('ready', 'FlyCo ready; waiting for the armed ground activation to take off')
                else:
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
        if self.auto_scan:
            self.tick_scan(now)

    def report(self, phase, message):
        self.phase = phase
        value = json.dumps(dict(phase=phase, initialized=self.initialized, target_z=self.target_z,
                               text=self.text, mesh_ready=self.mesh_ready, coverage=self.coverage, message=message))
        if value != self._last_status:
            self.status.publish(String(data=value))
            self.get_logger().info(value)
            self._last_status = value


def main():
    from flyco_common.execution import main_for
    main_for(FlyCoMission)
