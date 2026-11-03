"""Isaac rotor physics adapter for FlyCo's roll/pitch/yaw-rate/throttle output.

FlyCo's ROS position/velocity controller remains upstream. This replaces only
AirSim SimpleFlight's attitude actuator interface. Position hold/landing is a
local fallback for explicit operator actions or a lost upstream connection.
"""
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.backends import Backend


class FlyCoBackend(Backend):
    def __init__(self, full_thrust=40.):
        super().__init__(None)
        self.full_thrust = full_thrust
        self.mass = 1.5
        self.reset()

    def reset(self):
        self.mode = 'GROUNDED'
        self.state = None
        self.sim_time = 0.
        self.imu = None
        self.command = None
        self.received_wall = 0.
        self.yaw = 0.
        self.target = None
        self.ground_height = .062
        self.input_ref = [0.] * 4
        self.fault = ''
        self.command_count = 0
        self.last_thrust = 0.

    def start(self):
        self.reset()

    def stop(self):
        self.input_ref = [0.] * 4

    def update_state(self, state):
        self.state = state

    def update_sensor(self, sensor_type, data):
        if sensor_type == 'IMU':
            self.imu = data

    def update_graphical_sensor(self, sensor_type, data):
        pass

    def input_reference(self):
        return self.input_ref

    def set_command(self, command):
        if command and command.get('count', 0) != self.command_count:
            self.command = command
            self.command_count = command['count']
            self.received_wall = time.monotonic()

    def action(self, name):
        if self.state is None:
            return
        if name in ('initial', 'resume'):
            self.mode = 'WAITING'
            self.fault = ''
            self.yaw = Rotation.from_quat(self.state.attitude).as_euler('xyz')[2]
        elif name in ('hold', 'land'):
            self.mode = 'LANDING' if name == 'land' else 'HOLD'
            self.target = np.asarray(self.state.position).copy()
        elif name == 'disarm' and self.state.position[2] < self.ground_height + .12:
            self.mode = 'GROUNDED'
            self.input_ref = [0.] * 4

    def update(self, dt):
        self.sim_time += dt
        if self.state is None or self.mode == 'GROUNDED':
            self.input_ref = [0.] * 4
            return
        if self.mode == 'WAITING':
            if not self.command or time.monotonic()-self.received_wall > .2:
                self.input_ref = [0.] * 4
                return
            self.mode = 'EXTERNAL'
        p = np.asarray(self.state.position)
        v = np.asarray(self.state.linear_velocity)
        w = np.asarray(self.state.angular_velocity)
        r = Rotation.from_quat(self.state.attitude).as_matrix()
        if not all(np.all(np.isfinite(x)) for x in (p, v, w, r)):
            raise RuntimeError('Non-finite vehicle state')
        if r[2, 2] < math.cos(math.radians(70)):
            self.fault = 'Vehicle tipped over'
            self.mode = 'GROUNDED'
            self.input_ref = [0.] * 4
            return
        if self.mode == 'EXTERNAL' and (not self.command or
                time.monotonic()-self.received_wall > .75):
            self.action('hold')
            self.fault = 'Upstream control stale; holding until Resume'
        if self.mode == 'EXTERNAL':
            c = self.command
            roll, pitch, yaw_rate = c['rpy_rate']
            self.yaw += yaw_rate * dt
            desired = Rotation.from_euler('xyz', [roll, pitch, self.yaw]).as_matrix()
            thrust = np.clip(c['throttle'], 0., .7) * self.full_thrust
        else:
            if self.mode == 'LANDING':
                self.target[2] = max(self.ground_height-.06, self.target[2]-.3*dt)
                if p[2] < self.ground_height+.09 and np.linalg.norm(v) < .18:
                    self.mode = 'GROUNDED'
                    self.input_ref = [0.] * 4
                    return
            force = self.mass * (np.array([2.8, 2.8, 4.]) * (self.target-p)
                                 - np.array([3.6, 3.6, 4.]) * v + [0., 0., 9.81])
            force[2] = np.clip(force[2], .3*self.mass*9.81, 2*self.mass*9.81)
            force[:2] *= min(1., force[2]*.35/max(np.linalg.norm(force[:2]), 1e-9))
            z = force / np.linalg.norm(force)
            y = np.cross(z, [math.cos(self.yaw), math.sin(self.yaw), 0.])
            y /= np.linalg.norm(y)
            desired = np.column_stack((np.cross(y, z), y, z))
            thrust = np.clip(force @ r[:, 2], 0., self.full_thrust*.7)
            yaw_rate = 0.
        error = .5 * (desired.T@r-r.T@desired)
        e = np.array([error[2, 1], error[0, 2], error[1, 0]])
        torque = -np.array([3., 3., 1.])*e - np.array([.45, .45, .25])*(w-r.T@[0., 0., yaw_rate])
        torque = np.clip(torque, [-1.4, -1.4, -.45], [1.4, 1.4, .45])
        self.last_thrust = float(thrust)
        self.input_ref = np.asarray(self.vehicle.force_and_torques_to_velocities(
            float(thrust), torque)).tolist()
