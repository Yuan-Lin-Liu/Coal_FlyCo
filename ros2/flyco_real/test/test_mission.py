"""Automatic takeoff is caused by an armed ground activation, once per switch."""
import json
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from quadrotor_msgs.msg import PolynomialTrajGroup
from flyco_real.mission import FlyCoMission


class MissionFixture:
    def __init__(self):
        self.mission, self.provider = FlyCoMission(), Node('mission_test_provider')
        self.executor = SingleThreadedExecutor()
        for node in (self.mission, self.provider): self.executor.add_node(node)
        self.control = dict(mode='POSCTL', armed=True, enabled=True, landed=True, odom_ok=True,
            px4_ok=True, ground_ok=True, mode_executor_active=True, takeoff_phase='idle', takeoff_height=2.)
        self.z, self.requests, self.accept, self.planner = 0.1, 0, True, None
        self.joy = []
        self.status = self.provider.create_publisher(String, '/flyco/control/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom = self.provider.create_publisher(Odometry, '/flyco/odom', 10)
        self.gimbal = self.provider.create_publisher(QuaternionStamped, '/flyco/gimbal_pose', 10)
        self.provider.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.provider.create_service(Trigger, '/flyco/control/takeoff', self.takeoff)
        self.provider.create_timer(.02, self.publish)

    def on_joy(self, msg):
        self.joy.append(msg)
        if msg.buttons[4] and self.planner is None:
            self.planner = self.provider.create_publisher(PolynomialTrajGroup, '/local_newest_traj', 1)

    def takeoff(self, request, response):
        self.requests += 1
        response.success, response.message = self.accept, 'scheduled' if self.accept else 'not ready'
        if self.accept:
            self.control.update(mode='NAV_17', takeoff_phase='climbing', landed=False)
        return response

    def publish(self):
        stamp = self.provider.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id, odom.header.stamp = 'world', stamp
        odom.pose.pose.position.z, odom.pose.pose.orientation.w = self.z, 1.
        self.odom.publish(odom)
        gimbal = QuaternionStamped()
        gimbal.header.stamp, gimbal.quaternion.w = stamp, 1.
        self.gimbal.publish(gimbal)
        self.status.publish(String(data=json.dumps(self.control)))

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end: self.executor.spin_once(timeout_sec=.005)

    def initialize(self):
        self.spin(.5)
        assert self.mission.initialized
        assert any(msg.buttons[4] for msg in self.joy)
        assert list(self.joy[-1].buttons) == [0]*12

    def close(self):
        for node in (self.mission, self.provider):
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()


@pytest.fixture
def mission():
    rclpy.init()
    fixture = MissionFixture()
    try: yield fixture
    finally: fixture.close(); rclpy.shutdown()


def test_activation_takes_off_once_and_return_to_flyco_does_not_retrigger(mission):
    mission.initialize()
    assert mission.requests == 0
    mission.control['mode'] = 'FlyCo'
    mission.spin(.3)
    assert mission.requests == 1
    mission.control.update(mode='FlyCo', takeoff_phase='complete')
    mission.z = 2.1
    mission.spin(.4)
    assert mission.mission.phase == 'hovering'
    assert mission.requests == 1
    mission.control['mode'] = 'POSCTL'
    mission.spin(.1)
    mission.control['mode'] = 'FlyCo'
    mission.spin(.2)
    assert mission.requests == 1  # Already airborne: no second 2 m climb.


@pytest.mark.parametrize('condition', ['disarmed', 'airborne', 'unknown_landing_state'])
def test_activation_requires_normal_arming_and_known_ground_state(mission, condition):
    mission.initialize()
    if condition == 'disarmed': mission.control['armed'] = False
    elif condition == 'airborne': mission.control['landed'] = False
    else: mission.control['landed'] = None
    mission.control['mode'] = 'FlyCo'
    mission.spin(.3)
    assert mission.requests == 0


def test_starting_mission_in_flyco_requires_new_position_switch(mission):
    mission.control['mode'] = 'FlyCo'
    mission.initialize()
    assert mission.requests == 0
    mission.control['mode'] = 'POSCTL'
    mission.spin(.1)
    mission.control.update(mode='FlyCo', mode_executor_active=False)
    mission.spin(.2)
    assert mission.requests == 0
    mission.control['mode_executor_active'] = True
    mission.spin(.3)
    assert mission.requests == 1


def test_rejected_takeoff_is_reported_without_automatic_retries(mission):
    mission.accept = False
    mission.initialize()
    mission.control['mode'] = 'FlyCo'
    mission.spin(.5)
    assert mission.requests == 1
    assert mission.mission.phase == 'blocked'
    mission.spin(.3)
    assert mission.requests == 1
