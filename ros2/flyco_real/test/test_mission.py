"""Automatic takeoff is caused by an armed ground activation, once per switch."""
import json
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Joy, PointCloud2
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from quadrotor_msgs.msg import Mesh, PolynomialMatrix, PolynomialTrajGroup
from flyco_real.mission import FlyCoMission


class MissionFixture:
    def __init__(self, auto_scan=False):
        self.mission = FlyCoMission(parameter_overrides=[Parameter('auto_scan', value=auto_scan)])
        self.provider = Node('mission_test_provider')
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
        self.joy_subscription = self.provider.create_subscription(Joy, '/joy', self.on_joy, 10)
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


def test_combined_entry_waits_for_flyco_arming_and_does_not_repeat_on_return(mission):
    mission.mission.takeoff_on_armed_flyco = True
    mission.control.update(mode='FlyCo', armed=False)
    mission.initialize()
    assert mission.requests == 0
    mission.control['armed'] = True
    mission.spin(.3)
    assert mission.requests == 1
    mission.control.update(mode='FlyCo', takeoff_phase='complete', landed=False)
    mission.z = 2.1
    mission.spin(.3)
    assert mission.mission.phase == 'hovering'
    assert mission.requests == 1


def test_combined_entry_initializes_after_already_armed_ground_activation(mission):
    mission.mission.takeoff_on_armed_flyco = True
    mission.control['mode'] = 'FlyCo'
    mission.initialize()
    assert mission.requests == 1


@pytest.mark.parametrize('change', [dict(mode='POSCTL'), dict(armed=False), dict(landed=False)])
def test_combined_entry_cancels_pending_takeoff_when_ground_activation_is_lost(mission, change):
    mission.mission.takeoff_on_armed_flyco = True
    mission.control['mode'] = 'FlyCo'
    # Keep initialization pending long enough for the operator/ground-state change.
    mission.provider.destroy_subscription(mission.joy_subscription)
    mission.spin(.15)
    mission.control.update(change)
    mission.spin(.15)
    assert mission.requests == 0
    assert mission.mission.activation_deadline is None
    mission.planner = mission.provider.create_publisher(PolynomialTrajGroup, '/local_newest_traj', 1)
    mission.spin(.3)
    assert mission.mission.initialized
    assert mission.requests == 0


@pytest.fixture
def automatic_scan():
    rclpy.init()
    fixture = MissionFixture(auto_scan=True)
    fixture.scan_pubs = {name: fixture.provider.create_publisher(kind, topic, 10)
        for name, kind, topic in (
            ('text', String, '/query_text'), ('cloud', PointCloud2, '/tracking/target_pc'),
            ('mesh', Mesh, '/prediction/predicted_mesh'),
            ('finish', Bool, '/flight_end'))}
    fixture.resume_requests = fixture.hold_requests = 0
    def resume(request, response):
        fixture.resume_requests += 1
        fixture.control['tracking'] = True
        response.success = True
        return response
    def hold(request, response):
        fixture.hold_requests += 1
        fixture.control['tracking'] = False
        response.success = True
        return response
    fixture.provider.create_service(Trigger, '/flyco/control/resume', resume)
    fixture.provider.create_service(Trigger, '/flyco/control/hold', hold)
    try:
        fixture.initialize()
        fixture.scan_pubs['trajectory'] = fixture.planner
        fixture.z = 2.1
        fixture.control.update(mode='FlyCo', landed=False, tracking=False)
        fixture.spin(.15)
        fixture.mission.report('hovering', 'takeoff complete')
        fixture.spin(.15)
        yield fixture
    finally:
        fixture.close()
        rclpy.shutdown()


def ready_scan_target(fixture):
    fixture.scan_pubs['text'].publish(String(data='A pile of coal.'))
    fixture.spin(.1)
    cloud = PointCloud2(width=1, height=1)
    cloud.header.stamp = fixture.provider.get_clock().now().to_msg()
    fixture.scan_pubs['cloud'].publish(cloud)
    fixture.spin(.1)
    fixture.scan_pubs['mesh'].publish(Mesh(vertex_num=3, face_num=1))
    fixture.spin(.9)


def test_automatic_scan_requires_text_and_fresh_target_geometry(automatic_scan):
    f = automatic_scan
    f.scan_pubs['mesh'].publish(Mesh(vertex_num=3, face_num=1))
    f.spin(.1)
    assert f.mission.phase == 'waiting_for_text'
    f.scan_pubs['text'].publish(String(data='A pile of coal.'))
    f.spin(.1)
    stale = PointCloud2(width=1, height=1)
    f.scan_pubs['cloud'].publish(stale)
    f.scan_pubs['mesh'].publish(Mesh(vertex_num=3, face_num=1))
    f.spin(.1)
    assert f.mission.phase == 'waiting_for_mesh'
    assert not any(msg.buttons[2] or msg.buttons[5] for msg in f.joy)
    ready_scan_target(f)
    assert sum(bool(msg.buttons[2]) for msg in f.joy) == 1
    assert sum(bool(msg.buttons[5]) for msg in f.joy) == 1


def test_automatic_scan_tracks_and_finishes_once_with_position_hold(automatic_scan):
    f = automatic_scan
    ready_scan_target(f)
    trajectory = PolynomialTrajGroup(action=PolynomialTrajGroup.ACTION_ADD,
        start_time=f.provider.get_clock().now().to_msg(), pos_traj=[PolynomialMatrix(duration=1.)])
    f.scan_pubs['trajectory'].publish(trajectory)
    f.spin(.3)
    assert f.mission.phase == 'scanning'
    assert f.resume_requests == 1
    f.scan_pubs['finish'].publish(Bool(data=True))
    f.spin(.3)
    assert f.mission.phase == 'completed'
    assert f.hold_requests == 1
    assert not f.control['tracking']
    f.scan_pubs['text'].publish(String(data='A pile of coal.'))
    f.scan_pubs['mesh'].publish(Mesh(vertex_num=3, face_num=1))
    f.spin(.2)
    assert sum(bool(msg.buttons[5]) for msg in f.joy) == 1


@pytest.mark.parametrize('change', ['text', 'mode'])
def test_automatic_scan_stops_when_target_or_flight_mode_changes(automatic_scan, change):
    f = automatic_scan
    ready_scan_target(f)
    if change == 'text':
        f.scan_pubs['text'].publish(String(data='A different target.'))
    else:
        f.control['mode'] = 'POSCTL'
    f.spin(.2)
    assert f.mission.phase == 'blocked'
    assert f.hold_requests == 1
    assert f.resume_requests == 0
    if change == 'mode':
        f.control.update(mode='POSCTL', armed=False, landed=True)
        f.spin(.1)
        f.control.update(mode='FlyCo', armed=True)
        f.spin(.2)
        assert f.requests == 0
        assert f.mission.phase == 'blocked'
