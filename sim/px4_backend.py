"""Pegasus sensor/actuator transport plus FlyCo trajectory input for PX4 SITL.

Only PX4 supplies motor speeds. The independent UDP Offboard link leaves the
standard UDP 14550 ground-station link available to QGroundControl.
"""
import math
import time

import numpy as np
from pymavlink import mavutil
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, PX4MavlinkBackendConfig,
)


def ned(vector):
    x, y, z = vector
    return np.array([y, x, -z], dtype=float)


class WarehousePX4Backend(PX4MavlinkBackend):
    def __init__(self):
        super().__init__(PX4MavlinkBackendConfig({
            'px4_autolaunch': False, 'enable_lockstep': False,
            'connection_ip': '127.0.0.1', 'update_rate': 250.,
        }))
        self.link = mavutil.mavlink_connection(
            'udpin:127.0.0.1:14540', source_system=245, source_component=191)
        self.sim_time = 0.
        self.state = self.imu = None
        self.mass = 1.5
        self.ground_height = .062
        self.mode, self.fault = 'PX4_CONNECTING', ''
        self.command_count = 0
        self.command = None
        self.command_wall = self.heartbeat_wall = 0.
        self.local = self.offset = None
        self.armed = False
        self.px4_mode = 'UNKNOWN'
        self.target = None
        self.target_yaw = 0.
        self.active = self.tracking = False
        self.initialized = False
        self.accept_trajectories = self.endpoint_wait = False
        self.pending = None
        self.pending_since = 0.
        self.expected_command = None
        self.command_ack = None
        self.landing_state = 'idle'
        self.landing_attempts = 0
        self.landed_state = 0
        self.stream_started = self.last_send = self.last_hb = 0.
        self.ack = None
        self.setpoint_count = self.actuator_count = 0

    @property
    def input_ref(self):
        return self.input_reference()

    def update_sensor(self, sensor_type, data):
        super().update_sensor(sensor_type, data)
        if sensor_type == 'IMU':
            self.imu = data

    def update_state(self, state):
        self.state = state
        super().update_state(state)

    def handle_control(self, *args):
        self.actuator_count += 1
        return super().handle_control(*args)

    def command_long(self, command, *params):
        values = list(params) + [0.] * (7-len(params))
        self.link.mav.command_long_send(1, 1, command, 0, *values)
        self.expected_command = command
        self.command_ack = None
        self.pending_since = time.monotonic()

    def set_command(self, command):
        if command and command.get('count', 0) > self.command_count:
            fields = [*command['p'], *command['v'], *command['a'], command['yaw'], command['yaw_dot']]
            if np.all(np.isfinite(fields)):
                self.command = command
                self.command_count = command['count']
                self.command_wall = time.monotonic()
                # A normally completed segment keeps its endpoint and remains
                # eligible for the next segment. Hold/fault/takeover revoke this.
                if (self.accept_trajectories and self.endpoint_wait and
                        self.active and self.armed and self.px4_mode == 'OFFBOARD'
                        and self.pending is None):
                    self.tracking = True
                    self.endpoint_wait = False

    def capture_hold(self):
        if self.state is None:
            return False
        from scipy.spatial.transform import Rotation
        self.target = np.asarray(self.state.position).copy()
        self.target_yaw = float(Rotation.from_quat(self.state.attitude).as_euler('xyz')[2])
        return True

    def action(self, name):
        if name == 'initialize':
            if self.armed:
                self.fault = 'Already armed; use Hold or Resume'
                return
            if self.offset is None or not self.capture_hold():
                self.fault = 'PX4 local position not ready; retry Initialize'
                return
            self.active = True
            self.initialized = True
            self.accept_trajectories = self.endpoint_wait = self.tracking = False
            self.pending = None
            self.stream_started = time.monotonic()
            self.fault = ''
        elif name in ('initial', 'resume'):
            if not self.initialized or self.offset is None:
                self.fault = 'Initialize first; wait for PX4 local position'
                return
            if time.monotonic()-self.heartbeat_wall > 3.:
                self.fault = 'PX4 heartbeat unavailable; retry when connected'
                return
            # Simulation-only operator action: request Offboard, then normal arm.
            # QGC can always take over by changing mode. Never force-arm.
            self.active = True
            self.stream_started = self.pending_since = time.monotonic()
            self.pending = 'warmup'
            self.accept_trajectories = self.endpoint_wait = self.tracking = False
            self.landing_state = 'idle'
            self.expected_command = None
            self.command_wall = 0.
            self.command = None
            self.capture_hold()
            self.fault = ''
        elif name == 'hold':
            self.pending = None
            self.expected_command = None
            self.accept_trajectories = self.endpoint_wait = self.tracking = False
            self.capture_hold()
        elif name == 'land':
            self.accept_trajectories = self.endpoint_wait = self.tracking = False
            self.capture_hold()
            self.fault = ''
            self.landing_state = 'requested'
            self.landing_attempts = 0
            self.request_land()
        elif name == 'disarm':
            if self.state is not None and self.state.position[2] < self.ground_height + .15:
                self.pending = None
                self.active = self.accept_trajectories = self.endpoint_wait = self.tracking = False
                self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.)
            else:
                self.fault = 'Disarm rejected above ground; use Land'

    def request_land(self):
        self.pending = 'land'
        self.landing_attempts += 1
        # NaN latitude/longitude/altitude means land at the current location.
        # Keep streaming the captured hold until PX4 confirms LAND, avoiding
        # an Offboard-loss failsafe between sending the command and the switch.
        self.command_long(mavutil.mavlink.MAV_CMD_NAV_LAND,
                          0., 0., 0., math.nan, math.nan, math.nan, math.nan)

    def stop_tracking(self, fault):
        self.active = self.accept_trajectories = self.endpoint_wait = self.tracking = False
        self.pending = self.expected_command = None
        self.fault = fault

    @property
    def on_ground(self):
        return (self.state is not None and
                self.state.position[2] < self.ground_height + .15 and
                np.linalg.norm(self.state.linear_velocity) < .2)

    def update(self, dt):
        self.sim_time += dt
        super().update(dt)
        now = time.monotonic()
        while True:
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
                continue
            kind = msg.get_type()
            if kind == 'HEARTBEAT':
                previous = self.px4_mode
                was_armed = self.armed
                self.heartbeat_wall = now
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.px4_mode = {1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: {2: 'TAKEOFF', 3: 'LOITER', 4: 'MISSION', 5: 'RTL', 6: 'LAND'}.get((msg.custom_mode >> 24) & 255, 'AUTO'), 6: 'OFFBOARD', 7: 'STABILIZED', 8: 'RATTITUDE'}.get((msg.custom_mode >> 16) & 255, 'UNKNOWN')
                if self.px4_mode == 'LAND' and (previous == 'OFFBOARD' or
                        self.landing_state in ('requested', 'descending') or self.pending is None):
                    self.stop_tracking('')
                    self.landing_state = 'descending'
                elif previous == 'OFFBOARD' and self.px4_mode != 'OFFBOARD':
                    self.stop_tracking('PX4/QGC changed mode; Resume explicitly to return to FlyCo')
                elif was_armed and not self.armed:
                    self.stop_tracking('' if self.landing_state == 'descending'
                                       else 'PX4 disarmed; Resume explicitly to return to FlyCo')
            elif kind == 'LOCAL_POSITION_NED':
                self.local = np.array([msg.x, msg.y, msg.z])
                if not self.active and not self.armed and self.state is not None:
                    self.offset = ned(self.state.position)-self.local
            elif kind == 'COMMAND_ACK':
                # QGC has a separate system/component ID. Its acknowledgements
                # must never cancel an in-flight Offboard/arm/land request.
                if (getattr(msg, 'target_system', 245) not in (0, 245) or
                        getattr(msg, 'target_component', 191) not in (0, 191) or
                        msg.command != self.expected_command):
                    continue
                self.ack = {'command': msg.command, 'result': msg.result}
                self.command_ack = msg.result
                if msg.result not in (0, 5):
                    self.fault = f'PX4 rejected command {msg.command}: result={msg.result}; inspect QGC'
                    self.pending = None
                    self.accept_trajectories = self.endpoint_wait = self.tracking = False
                    if self.landing_state == 'requested':
                        self.landing_state = 'failed'
            elif kind == 'EXTENDED_SYS_STATE':
                self.landed_state = msg.landed_state
            elif kind == 'STATUSTEXT':
                print('[PX4] '+msg.text, flush=True)
        if now-self.last_hb >= 1. and self.link.address:
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_hb = now
        if self.active and now-self.heartbeat_wall > 3.:
            self.stop_tracking('PX4 heartbeat lost; stopped Offboard stream')
        if self.active and self.offset is not None and now-self.last_send >= .02:
            use_command = self.tracking and self.command is not None and now-self.command_wall < .75
            if self.tracking and now-self.command_wall >= (15. if self.command is None else .75):
                self.tracking = False
                # traj_server stops publishing at a completed polynomial's
                # zero-velocity endpoint. Keep that endpoint while PX4 settles.
                end = self.command.get('trajectory_end') if self.command is not None else None
                completed = (self.command is not None and
                             (end is None or self.command.get('t', -math.inf) >= end-.12) and
                             np.linalg.norm(self.command['v']) < .05 and
                             np.linalg.norm(self.command['a']) < .1)
                if completed:
                    self.target = np.asarray(self.command['p']).copy()
                    self.target_yaw = self.command['yaw']
                    self.fault = ''
                    self.endpoint_wait = self.accept_trajectories
                else:
                    self.accept_trajectories = self.endpoint_wait = False
                    self.capture_hold()
                    self.fault = 'FlyCo trajectory stale; holding until Resume'
            c = self.command if use_command else dict(p=self.target, v=[0., 0., 0.],
                a=[0., 0., 0.], yaw=self.target_yaw, yaw_dot=0.)
            p = ned(c['p'])-self.offset
            v, a = ned(c['v']), ned(c['a'])
            yaw = (math.pi/2-c['yaw']+math.pi)%(2*math.pi)-math.pi
            self.link.mav.set_position_target_local_ned_send(
                int(self.sim_time*1000)%(2**32), 1, 1,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0,
                *p, *v, *a, yaw, -c['yaw_dot'])
            self.last_send = now
            self.setpoint_count += 1
        if self.pending == 'land':
            if now-self.pending_since > 3.:
                if self.landing_attempts < 3:
                    self.request_land()
                else:
                    self.pending = None
                    self.landing_state = 'failed'
                    self.fault = 'PX4 LAND mode not confirmed; inspect QGC and retry Land'
        elif self.pending and self.active:
            if self.pending == 'warmup' and now-self.stream_started > 1.2:
                self.command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1., 6.)
                self.pending = 'wait_mode'
            elif self.pending == 'wait_mode' and self.px4_mode == 'OFFBOARD':
                if not self.armed:
                    self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.)
                else:
                    self.pending_since = now
                self.pending = 'wait_arm'
            elif self.pending == 'wait_arm' and self.armed:
                self.pending = None
                self.expected_command = None
                self.tracking = True
                self.accept_trajectories = True
                if self.command is None:
                    self.command_wall = now  # grace while first trajectory arrives
            elif now-self.pending_since > 6.:
                self.pending = self.expected_command = None
                self.fault = 'PX4 mode/arm confirmation timed out; inspect QGC and retry Resume'
        if (self.landing_state in ('requested', 'descending') and not self.armed
                and self.on_ground and max(map(abs, self.input_ref), default=0.) < 1.):
            self.stop_tracking('')
            self.landing_state = 'complete'
        self.mode = ('PX4_'+self.px4_mode+('_ARMED' if self.armed else '_DISARMED'))

    def telemetry(self):
        return dict(armed=self.armed, flight_mode=self.px4_mode,
            heartbeat_age=time.monotonic()-self.heartbeat_wall,
            local_position_ned=self.local.tolist() if self.local is not None else None,
            world_to_local_offset=self.offset.tolist() if self.offset is not None else None,
            offboard_stream=self.active, tracking=self.tracking, pending=self.pending,
            accept_trajectories=self.accept_trajectories, endpoint_wait=self.endpoint_wait,
            landing_state=self.landing_state, landing_complete=self.landing_state == 'complete',
            landed_state=self.landed_state,
            setpoints=self.setpoint_count, actuator_messages=self.actuator_count, last_ack=self.ack)

    def stop(self):
        self.link.close()
        super().stop()
