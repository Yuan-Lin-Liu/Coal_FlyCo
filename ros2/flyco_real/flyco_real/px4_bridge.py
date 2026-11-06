"""FlyCo trajectories to PX4 native DDS, with explicit operator engagement."""
import json
import math
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (TrajectorySetpoint, VehicleLocalPosition,
                         VehicleStatus, VehicleCommand, VehicleCommandAck, ModeCompleted, VehicleLandDetected)
from quadrotor_msgs.msg import PositionCommand, PolynomialTrajGroup
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool, Trigger
from .control import fresh, validate_command
from .geometry import pose_matrix, world_alignment
from .px4_dds import ENU_TO_NED, local_pose_enu, topic, yaw_ned
from .mode_registration import FlyCoMode


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.e-9


def xyz(value):
    return np.array([value.x, value.y, value.z])


class PX4Bridge(Node):
    def __init__(self):
        super().__init__('px4_bridge', automatically_declare_parameters_from_overrides=True)
        self.settings = {}
        for key, default in {
            'odom_topic': '/flyco/odom', 'command_topic': '/planning/pos_cmd',
            'px4_topic_prefix': '/fmu', 'world_frame': 'world', 'rate': 50.,
            'odom_timeout': .3, 'state_timeout': 2., 'command_timeout': .3,
            'warmup_seconds': 1.2, 'max_speed': 1.5, 'max_acceleration': 3.,
            'max_yaw_rate': 1., 'max_position_error': 2.,
            'bounds_min': [-30., -30., -.5], 'bounds_max': [30., 30., 10.],
            'enable_control': False, 'ground_link_timeout': 5., 'first_command_timeout': 15.,
            'auto_enable': False,
            'land_state_timeout': 5.,
            'target_system': 0, 'target_component': 0, 'source_component': 191,
            'command_ack_timeout': 3.,
            'simulation': False,
            'takeoff_height': 2., 'takeoff_timeout': 60.,
            'max_heading_reset': .35,
        }.items():
            if not self.has_parameter(key): self.declare_parameter(key, default)
            self.settings[key] = self.get_parameter(key).value
        if self.settings['enable_control']:
            raise ValueError('Use task.py enable; startup cannot engage control')
        if self.settings['rate'] < 10. or self.settings['warmup_seconds'] < 1.:
            raise ValueError('FlyCo requires at least 10 Hz output and one second of warmup')
        if (len(self.settings['bounds_min']) != 3 or len(self.settings['bounds_max']) != 3 or
                np.any(np.array(self.settings['bounds_min']) >= self.settings['bounds_max'])):
            raise ValueError('Invalid flight volume')
        self.odom = self.local = self.command = self.state = None
        self.odom_rx = self.local_rx = self.state_rx = self.cmd_rx = None
        self.enabled = self.seen_flyco = self.tracking = False
        self.hold = self.alignment = self.enabled_at = None
        self.reason = 'disabled'
        self.ground_rx = None
        self._last_status = None
        self._last_command_stamp = -math.inf
        self.land_pending = None
        self.land_result = 'none'
        self.endpoint_wait = False
        self.trajectory_windows = []
        self.takeoff_phase = 'idle'
        self.takeoff_started = 0.
        self.takeoff_world_target = None
        self.estimator_reset = None
        self.auto_enable_pending = bool(self.settings['auto_enable'])
        self.auto_enable_inhibited = False
        self.auto_enable_ready_since = None
        self.land_detected = self.land_detected_rx = None
        prefix = self.settings['px4_topic_prefix']
        self.state_topic = topic(prefix, 'out', 'vehicle_status', VehicleStatus)
        self.local_topic = topic(prefix, 'out', 'vehicle_local_position', VehicleLocalPosition)
        self.output = self.create_publisher(TrajectorySetpoint, topic(prefix, 'in', 'trajectory_setpoint', TrajectorySetpoint), 1)
        self.flight_mode = FlyCoMode(self, prefix, lambda: self.enabled and self.ready() and
            fresh(self.ground_rx, time.monotonic(), self.settings['ground_link_timeout']))
        self.vehicle_command = self.create_publisher(VehicleCommand, topic(prefix, 'in', 'vehicle_command', VehicleCommand), 1)
        self.executor_command = self.create_publisher(VehicleCommand,
            topic(prefix, 'in', 'vehicle_command_mode_executor', VehicleCommand), 1)
        self.preview = self.create_publisher(TrajectorySetpoint, '/flyco/control/preview', 1)
        self.status = self.create_publisher(String, '/flyco/control/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, self.settings['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, self.local_topic, self.on_local, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, self.state_topic, self.on_state, qos_profile_sensor_data)
        self.create_subscription(VehicleLandDetected,
            topic(prefix, 'out', 'vehicle_land_detected', VehicleLandDetected), self.on_land_detected,
            qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck, topic(prefix, 'out', 'vehicle_command_ack', VehicleCommandAck), self.on_ack, qos_profile_sensor_data)
        self.create_subscription(PositionCommand, self.settings['command_topic'], self.on_command, 10)
        self.create_subscription(PolynomialTrajGroup, '/local_newest_traj', self.on_trajectory, 10)
        self.create_subscription(ModeCompleted, topic(prefix, 'out', 'mode_completed', ModeCompleted),
                                 self.on_mode_completed, qos_profile_sensor_data)
        self.create_subscription(Empty, '/flyco/ground/heartbeat', self.on_ground, 1)
        self.create_service(SetBool, '/flyco/control/enable', self.enable)
        self.create_service(Trigger, '/flyco/control/hold', self.hold_service)
        self.create_service(Trigger, '/flyco/control/resume', self.resume)
        self.create_service(Trigger, '/flyco/control/land', self.land)
        self.create_service(Trigger, '/flyco/control/takeoff', self.takeoff)
        self.timer = self.create_timer(1. / self.settings['rate'], self.tick)
        self.get_logger().info('PX4 DDS ready, outputs disabled; telemetry: %s, %s' % (self.state_topic, self.local_topic))

    def now(self):
        return self.get_clock().now().nanoseconds * 1.e-9

    def timestamp_us(self):
        # PX4 uXRCE-DDS time sync maps the ROS clock to the FCU clock.
        if self.settings['simulation'] and self.local is not None:
            return int(self.local.timestamp)
        return self.get_clock().now().nanoseconds // 1000

    def px4_now(self):
        # SITL without XRCE time sync has a sensor clock whose epoch differs
        # from Isaac /clock. Receive-age checks still enforce link freshness.
        if self.settings['simulation'] and self.local is not None:
            return self.local.timestamp*1.e-6
        return self.now()

    def on_odom(self, msg):
        try:
            if msg.header.frame_id != self.settings['world_frame']: return
            pose_matrix(msg.pose.pose)
        except ValueError: return
        self.odom, self.odom_rx = msg, time.monotonic()

    def on_local(self, msg):
        if self.local and msg.timestamp <= self.local.timestamp: return
        old = self.local
        self.local, self.local_rx = msg, time.monotonic()
        counters = ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter')
        if self.enabled and old and any(getattr(old, key) != getattr(msg, key) for key in counters):
            changed = {key: [int(getattr(old, key)), int(getattr(msg, key))]
                       for key in counters if getattr(old, key) != getattr(msg, key)}
            self.estimator_reset = dict(counters=changed,
                delta_xy=list(map(float, msg.delta_xy)), delta_z=float(msg.delta_z),
                delta_heading=float(msg.delta_heading),
                before=[float(old.x), float(old.y), float(old.z), float(old.heading)],
                after=[float(msg.x), float(msg.y), float(msg.z), float(msg.heading)])
            self.get_logger().error('PX4 estimator reset: '+json.dumps(self.estimator_reset))
            # EKF yaw finalization can report a small, explicit heading-only
            # reset after takeoff. Re-anchor the world transform at the current
            # measured pose; position resets and unknown jumps still stop control.
            if (set(changed) == {'heading_reset_counter'} and self.alignment is not None
                    and self.odom_ok() and np.isfinite(msg.delta_heading)
                    and abs(msg.delta_heading) <= self.settings['max_heading_reset']
                    and np.isfinite([msg.x, msg.y, msg.z, msg.heading]).all()):
                previous_rotation, previous_offset, previous_yaw = self.alignment
                world_hold = (previous_rotation.T @ (ENU_TO_NED @ self.hold[0]-previous_offset)
                              if self.hold is not None else None)
                rotation, offset, yaw = world_alignment(pose_matrix(self.odom.pose.pose), local_pose_enu(msg))
                self.alignment = rotation, offset, yaw
                if world_hold is not None:
                    self.hold = (ENU_TO_NED @ (rotation @ world_hold+offset),
                                 yaw_ned(yaw_ned(self.hold[1])-previous_yaw+yaw))
                self.reason = 'PX4 heading-only reset compensated from fresh world pose'
                return
            self.enabled = False
            self.latch_hold('PX4 estimator reset; inspect frames and enable again')

    def on_state(self, msg):
        if self.state and msg.timestamp <= self.state.timestamp: return
        previous = self.state
        self.state, self.state_rx = msg, time.monotonic()
        if (self.settings['auto_enable'] and not self.auto_enable_inhibited and previous and
                previous.nav_state != msg.nav_state and msg.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL):
            # An explicit return to Position allows a new engagement. Sensor
            # failures in FlyCo do not silently re-enable themselves.
            self.auto_enable_pending = self.seen_flyco or not self.enabled
            self.auto_enable_ready_since = None
        if (previous and previous.nav_state != msg.nav_state and
                msg.nav_state == self.flight_mode.mode_id and self.enabled and self.ready() and
                self.takeoff_phase not in ('requested', 'climbing', 'returning')):
            self.latch_hold('FlyCo selected; holding the current position')

    def on_land_detected(self, msg):
        if self.land_detected and msg.timestamp <= self.land_detected.timestamp: return
        self.land_detected, self.land_detected_rx = msg, time.monotonic()

    def on_command(self, msg):
        if (msg.header.frame_id != self.settings['world_frame'] or
                msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY): return
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= self._last_command_stamp or not fresh(stamp, self.now(), self.settings['command_timeout']): return
        self.command, self.cmd_rx = msg, time.monotonic()
        self._last_command_stamp = stamp
        if self.endpoint_wait and self.enabled and self.armed_flyco():
            self.tracking, self.endpoint_wait, self.reason = True, False, 'tracking next segment'

    def on_trajectory(self, msg):
        if msg.action != PolynomialTrajGroup.ACTION_ADD or not msg.pos_traj:
            return
        start = stamp_seconds(msg.start_time)
        self.trajectory_windows = (self.trajectory_windows +
            [(start, start+sum(piece.duration for piece in msg.pos_traj))])[-8:]

    def on_ground(self, msg):
        self.ground_rx = time.monotonic()

    def odom_ok(self):
        return bool(self.odom and fresh(self.odom_rx, time.monotonic(), self.settings['odom_timeout']) and
                    fresh(stamp_seconds(self.odom.header.stamp), self.now(), self.settings['odom_timeout']))

    def px4_issue(self):
        if not self.state or not fresh(self.state_rx, time.monotonic(), self.settings['state_timeout']):
            return 'waiting for ' + self.state_topic
        if self.state.failsafe:
            return 'PX4 failsafe is active'
        if not fresh(self.state.timestamp * 1.e-6, self.px4_now(), self.settings['state_timeout']):
            return 'PX4 status clock is not synchronized with ROS'
        if not self.local or not fresh(self.local_rx, time.monotonic(), self.settings['odom_timeout']):
            return 'waiting for ' + self.local_topic
        if not fresh(self.local.timestamp * 1.e-6, self.px4_now(), self.settings['odom_timeout']):
            return 'PX4 local position clock is not synchronized with ROS'
        # PX4's registered attitude/local-position mode requirements own EKF
        # arming and in-flight health checks. heading_good_for_control is an
        # estimator final-alignment hint, not the external-mode validity API.
        if (not self.local.xy_valid or not self.local.z_valid or
                not np.isfinite([self.local.x, self.local.y, self.local.z, self.local.heading]).all()):
            return 'PX4 local position/heading is invalid'
        return ''

    def ready(self):
        return self.odom_ok() and not self.px4_issue()

    def armed_flyco(self):
        return bool(self.state and self.state.arming_state == VehicleStatus.ARMING_STATE_ARMED and
                    self.flight_mode.mode_id is not None and self.state.nav_state == self.flight_mode.mode_id)

    def latch_hold(self, reason):
        self.endpoint_wait = False
        if self.local and np.isfinite([self.local.x,self.local.y,self.local.z,self.local.heading]).all():
            self.hold = (np.array([self.local.x,self.local.y,self.local.z]), float(self.local.heading))
        self.tracking, self.reason = False, reason

    def enable(self, request, response):
        if not request.data:
            self.enabled = self.tracking = False
            self.auto_enable_pending = False
            self.auto_enable_inhibited = True
            self.reason = 'disabled by operator'
            response.success, response.message = True, self.reason
            return response
        if self.enabled:
            response.success, response.message = False, 'Already enabled; use resume'
            return response
        if not self.ready():
            response.success, response.message = False, self.px4_issue() or 'Need fresh /flyco/odom'
            return response
        self.alignment = world_alignment(pose_matrix(self.odom.pose.pose), local_pose_enu(self.local))
        self.enabled, self.seen_flyco = True, False
        self.auto_enable_pending = False
        self.auto_enable_inhibited = False
        self.auto_enable_ready_since = None
        self.enabled_at = time.monotonic()
        self.command = None
        self.latch_hold('ready; normally arm on the ground in Position, then select FlyCo')
        response.success, response.message = True, self.reason
        return response

    def hold_service(self, request, response):
        self.latch_hold('operator hold')
        response.success, response.message = self.enabled and self.ready(), self.reason
        return response

    def executor_request(self, command, **params):
        msg = VehicleCommand(command=command, timestamp=0,
            source_component=VehicleCommand.COMPONENT_MODE_EXECUTOR_START+self.flight_mode.executor_id,
            **params)
        self.executor_command.publish(msg)

    def takeoff(self, request, response):
        if not (self.enabled and self.ready() and self.armed_flyco() and
                self.flight_mode.executor_id is not None and
                self.state.executor_in_charge == self.flight_mode.executor_id):
            response.success, response.message = False, 'Enable, select FlyCo and normally arm before takeoff'
            return response
        if self.takeoff_phase in ('requested', 'climbing', 'returning'):
            response.success, response.message = False, 'Takeoff is already in progress'
            return response
        height = float(self.settings['takeoff_height'])
        if not (0. < height <= 5. and np.isfinite(self.local.ref_alt) and
                xyz(self.odom.pose.pose.position)[2]+height <= self.settings['bounds_max'][2]):
            response.success, response.message = False, 'Invalid takeoff altitude/reference or flight volume'
            return response
        self.latch_hold('PX4 native takeoff scheduled')
        self.takeoff_phase, self.takeoff_started = 'requested', time.monotonic()
        self.takeoff_world_target = xyz(self.odom.pose.pose.position)+[0., 0., height]
        self.executor_request(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=math.nan, param2=math.nan, param3=math.nan, param4=math.nan,
            param5=math.nan, param6=math.nan, param7=float(self.local.ref_alt-self.local.z+height))
        response.success, response.message = True, 'PX4 Takeoff scheduled; returns to FlyCo on completion'
        return response

    def on_mode_completed(self, msg):
        if self.takeoff_phase not in ('requested', 'climbing') or msg.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            return
        if msg.result != ModeCompleted.RESULT_SUCCESS:
            self.takeoff_phase = 'failed'
            self.enabled = False
            self.latch_hold('PX4 takeoff failed; inspect mode and land using RC/QGC')
            return
        self.takeoff_phase = 'returning'
        self.executor_request(VehicleCommand.VEHICLE_CMD_SET_NAV_STATE, param1=float(self.flight_mode.mode_id))

    def resume(self, request, response):
        if not (self.enabled and self.ready() and self.armed_flyco() and
                time.monotonic() - self.enabled_at >= self.settings['warmup_seconds']):
            response.success, response.message = False, 'Enable and warm up; select FlyCo and arm explicitly first'
            return response
        if not fresh(self.ground_rx, time.monotonic(), self.settings['ground_link_timeout']):
            response.success, response.message = False, 'Ground heartbeat is missing'
            return response
        self.tracking, self.reason = True, 'tracking'
        self.command = self.cmd_rx = None
        self.received_active_command = False
        self.resume_at = time.monotonic()
        response.success, response.message = True, 'Accepting fresh FlyCo commands'
        return response

    def land(self, request, response):
        if (not self.state or not fresh(self.state_rx, time.monotonic(), self.settings['state_timeout']) or
                not fresh(self.state.timestamp * 1.e-6, self.px4_now(), self.settings['state_timeout'])):
            response.success, response.message = False, 'PX4 status is unavailable'
            return response
        if self.land_pending:
            response.success, response.message = False, 'Waiting for the previous landing acknowledgment'
            return response
        cmd = VehicleCommand()
        cmd.timestamp = self.timestamp_us()
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd.param4 = cmd.param5 = cmd.param6 = cmd.param7 = math.nan
        cmd.target_system = int(self.settings['target_system'] or self.state.system_id or 1)
        cmd.target_component = int(self.settings['target_component'] or self.state.component_id or 1)
        cmd.source_system, cmd.source_component = cmd.target_system, int(self.settings['source_component'])
        cmd.from_external = True
        self.land_pending = (time.monotonic(), cmd.timestamp, cmd.source_system, cmd.source_component)
        self.land_result = 'pending'
        self.latch_hold('landing requested; waiting for PX4 acknowledgment')
        self.vehicle_command.publish(cmd)
        response.success, response.message = True, 'Landing requested; confirm land_result and PX4 mode'
        return response

    def on_ack(self, msg):
        if not self.land_pending: return
        _, stamp, system, component = self.land_pending
        if (msg.command != VehicleCommand.VEHICLE_CMD_NAV_LAND or msg.timestamp + 50000 < stamp or
                msg.target_system != system or msg.target_component != component or
                not fresh(msg.timestamp * 1.e-6, self.px4_now(), self.settings['command_ack_timeout'])): return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: return
        self.land_result = 'accepted' if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED else 'rejected (%d)' % msg.result
        self.land_pending = None

    def make_hold(self):
        msg = TrajectorySetpoint()
        msg.position = self.hold[0].astype(float).tolist()
        msg.yaw = self.hold[1]
        msg.velocity = msg.acceleration = msg.jerk = [math.nan] * 3
        msg.yawspeed = math.nan
        return msg

    def tick(self):
        self.flight_mode.tick()
        if self.auto_enable_pending and not self.enabled:
            can_enable = (self.flight_mode.mode_id is not None and self.ready() and
                fresh(self.ground_rx, time.monotonic(), self.settings['ground_link_timeout']) and
                self.state.nav_state != self.flight_mode.mode_id and
                (self.state.arming_state != VehicleStatus.ARMING_STATE_ARMED or
                 self.state.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL))
            if not can_enable:
                self.auto_enable_ready_since = None
            else:
                if self.auto_enable_ready_since is None:
                    self.auto_enable_ready_since = time.monotonic()
                if time.monotonic()-self.auto_enable_ready_since >= self.settings['warmup_seconds']:
                    self.enable(SetBool.Request(data=True), SetBool.Response())
        taking_off = self.takeoff_phase in ('requested', 'climbing', 'returning')
        if taking_off:
            owns = self.state and self.state.executor_in_charge == self.flight_mode.executor_id
            expected = self.state and self.state.nav_state in (
                self.flight_mode.mode_id, VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF)
            if not owns or not expected or time.monotonic()-self.takeoff_started > self.settings['takeoff_timeout']:
                self.takeoff_phase = 'failed'
                self.enabled = False
                self.latch_hold('Takeoff cancelled, mode taken over or timed out; inspect RC/QGC')
                taking_off = False
            elif (self.state.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF and
                  self.takeoff_phase == 'requested'):
                self.takeoff_phase = 'climbing'
            elif self.takeoff_phase == 'returning' and self.armed_flyco():
                self.takeoff_phase = 'complete'
                self.latch_hold('Takeoff complete; holding in FlyCo')
                rotation, offset, _ = self.alignment
                self.hold = (ENU_TO_NED @ (rotation @ self.takeoff_world_target+offset), self.hold[1])
                self.endpoint_wait = True
                taking_off = False
        if self.land_pending and time.monotonic() - self.land_pending[0] > self.settings['command_ack_timeout']:
            self.land_pending = None
            self.land_result = 'ack timeout; inspect PX4 mode'
        if self.enabled and not self.ready():
            self.enabled = False
            self.latch_hold('state/odometry lost; FlyCo output stopped')
        if self.enabled and self.seen_flyco and not self.armed_flyco() and not taking_off:
            self.enabled = False
            self.latch_hold('PX4 left armed FlyCo; enable again after inspection')
        if self.enabled and self.armed_flyco(): self.seen_flyco = True
        if taking_off:
            self.publish_status()
            return
        if not self.enabled or self.hold is None:
            self.publish_status()
            return
        target = self.make_hold()
        if self.tracking:
            now = time.monotonic()
            if not fresh(self.ground_rx, now, self.settings['ground_link_timeout']):
                self.latch_hold('ground link lost; resume after restoration')
            if not (self.command and fresh(self.cmd_rx, now, self.settings['command_timeout']) and
                    fresh(stamp_seconds(self.command.header.stamp), self.now(), self.settings['command_timeout'])):
                timeout = self.settings['command_timeout'] if self.received_active_command else self.settings['first_command_timeout']
                if now - (self.cmd_rx or self.resume_at) > timeout:
                    windows = [w for w in self.trajectory_windows if self.command and
                               w[0] <= stamp_seconds(self.command.header.stamp)]
                    end = max(windows)[1] if windows else math.inf
                    completed = (self.command and stamp_seconds(self.command.header.stamp) >= end-.12 and
                                 np.linalg.norm(xyz(self.command.velocity)) < .05 and
                                 np.linalg.norm(xyz(self.command.acceleration)) < .1)
                    self.latch_hold('trajectory endpoint' if completed else 'trajectory timeout; resume after restoration')
                    if completed:
                        rotation, offset, yaw_offset = self.alignment
                        self.hold = (ENU_TO_NED @ (rotation @ xyz(self.command.position)+offset),
                                     yaw_ned(self.command.yaw+yaw_offset))
                        self.endpoint_wait = True
            elif self.tracking:
                self.received_active_command = True
                cmd = self.command
                p,v,a = xyz(cmd.position), xyz(cmd.velocity), xyz(cmd.acceleration)
                reason = validate_command(p,v,a,cmd.yaw,cmd.yaw_dot,xyz(self.odom.pose.pose.position),
                    self.settings['bounds_min'],self.settings['bounds_max'],self.settings['max_speed'],
                    self.settings['max_acceleration'],self.settings['max_yaw_rate'],self.settings['max_position_error'])
                rotation,offset,yaw_offset = self.alignment
                if np.linalg.norm(rotation @ xyz(self.odom.pose.pose.position) + offset - local_pose_enu(self.local)[:3,3]) > .75:
                    reason = 'PX4/Odin local frames diverged'
                if reason: self.latch_hold(reason)
                else:
                    target.position = (ENU_TO_NED @ (rotation @ p + offset)).astype(float).tolist()
                    target.velocity = (ENU_TO_NED @ rotation @ v).astype(float).tolist()
                    target.acceleration = (ENU_TO_NED @ rotation @ a).astype(float).tolist()
                    target.yaw = yaw_ned(cmd.yaw + yaw_offset)
                    target.yawspeed = -float(cmd.yaw_dot)
        if not self.tracking: target = self.make_hold()
        target.timestamp = self.timestamp_us()
        # Registered external modes supply the requirements through
        # config_control_setpoints; they never emit OffboardControlMode.
        if self.flight_mode.mode_id is not None and self.state and self.state.nav_state == self.flight_mode.mode_id:
            self.output.publish(target)
        self.preview.publish(target)
        self.publish_status()

    def publish_status(self):
        names = {VehicleStatus.NAVIGATION_STATE_OFFBOARD:'OFFBOARD', VehicleStatus.NAVIGATION_STATE_POSCTL:'POSCTL',
                 VehicleStatus.NAVIGATION_STATE_AUTO_LAND:'AUTO.LAND'}
        issue = self.px4_issue()
        value = json.dumps({'enabled':self.enabled,'tracking':self.tracking,
            'auto_enable_pending': self.auto_enable_pending,
            'landed': (bool(self.land_detected.landed) if self.land_detected and
                       fresh(self.land_detected_rx, time.monotonic(), self.settings['land_state_timeout']) else None),
            'mode_executor_active': bool(self.state and self.flight_mode.executor_id is not None and
                                        self.state.executor_in_charge == self.flight_mode.executor_id),
            'endpoint_wait': self.endpoint_wait,
            'takeoff_phase': self.takeoff_phase, 'mode_executor_id': self.flight_mode.executor_id,
            'takeoff_height': self.settings['takeoff_height'],
            'estimator_reset': self.estimator_reset,
            'mode': ('FlyCo' if self.flight_mode.mode_id is not None and self.state.nav_state == self.flight_mode.mode_id
                     else names.get(self.state.nav_state,'NAV_%d' % self.state.nav_state)) if self.state else 'unknown',
            'registered_mode_id': self.flight_mode.mode_id, 'registration_issue': self.flight_mode.error,
            'armed': bool(self.state and self.state.arming_state == VehicleStatus.ARMING_STATE_ARMED),
            'odom_ok':self.odom_ok(),'px4_ok':not bool(issue),
            'ground_ok':fresh(self.ground_rx,time.monotonic(),self.settings['ground_link_timeout']),
            'px4_issue':issue,'reason':self.reason,'land_result':self.land_result})
        if value != self._last_status:
            self.status.publish(String(data=value))
            self.get_logger().info(value)
            self._last_status = value

    def destroy_node(self):
        self.flight_mode.close()
        return super().destroy_node()


def main():
    from flyco_common.execution import main_for
    main_for(PX4Bridge)
