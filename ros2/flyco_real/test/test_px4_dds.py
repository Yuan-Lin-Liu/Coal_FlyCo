"""Exercise actual PX4 DDS message contracts without a flight controller."""
import json
import math
import time
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleOdometry, VehicleCommand,
                         VehicleCommandAck, TrajectorySetpoint, OffboardControlMode,
                         RegisterExtComponentRequest, RegisterExtComponentReply, VehicleControlMode, ModeCompleted)
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger
from flyco_real.px4_bridge import PX4Bridge
from flyco_real.px4_dds import topic, visual_odometry


def test_px4_frames_covariance_and_versioned_names():
    assert topic('/uav1/fmu','out','vehicle_status',VehicleStatus) == '/uav1/fmu/out/vehicle_status_v1'
    assert topic('/fmu','in','offboard_control_mode',OffboardControlMode) == '/fmu/in/offboard_control_mode'
    source=Odometry()
    source.header.stamp.sec=123; source.header.stamp.nanosec=456789000
    source.pose.pose.position.x,source.pose.pose.position.y,source.pose.pose.position.z=1.,2.,3.
    source.pose.pose.orientation.z=source.pose.pose.orientation.w=math.sqrt(.5)
    source.pose.covariance=np.diag([.1,.2,.3,.4,.5,.6]).flatten().tolist()
    target=visual_odometry(source,123500000)
    assert target.timestamp==123500000 and target.timestamp_sample==123456789
    assert visual_odometry(source,123500000,VehicleOdometry.POSE_FRAME_NED).pose_frame == VehicleOdometry.POSE_FRAME_NED
    assert target.pose_frame==VehicleOdometry.POSE_FRAME_FRD
    assert np.allclose(target.position,[2.,1.,-3.])
    assert np.allclose(target.q,[1.,0.,0.,0.],atol=1.e-6)  # ENU North-facing FLU -> NED/FRD identity.
    assert np.allclose(target.position_variance,[.2,.1,.3])
    assert np.allclose(target.orientation_variance,[.5,.4,.6])
    assert target.velocity_frame==VehicleOdometry.VELOCITY_FRAME_UNKNOWN
    assert np.isnan(target.velocity).all() and np.isnan(target.velocity_variance).all()
    source.pose.pose.orientation.z=0.; source.pose.pose.orientation.w=1.
    target=visual_odometry(source,123500000)
    assert np.allclose(target.q,[math.sqrt(.5),0.,0.,math.sqrt(.5)],atol=1.e-6)


class DDSFixture:
    def __init__(self):
        self.bridge,self.node=PX4Bridge(),Node('fake_px4_dds')
        self.executor=SingleThreadedExecutor()
        for node in (self.bridge,self.node): self.executor.add_node(node)
        self.targets,self.heartbeats,self.requests=[],[],[]
        self.live=dict(odom=True,local=True,state=True,ground=True,command=None,armed=False,
                       mode=VehicleStatus.NAVIGATION_STATE_POSCTL,valid=True,reset=0,clock_offset=0.,
                       ack=VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,wrong_ack_target=False,ack_future=False,failsafe=False)
        self.odom=self.node.create_publisher(Odometry,'/flyco/odom',qos_profile_sensor_data)
        self.local=self.node.create_publisher(VehicleLocalPosition,'/fmu/out/vehicle_local_position',qos_profile_sensor_data)
        self.state=self.node.create_publisher(VehicleStatus,'/fmu/out/vehicle_status_v1',qos_profile_sensor_data)
        self.ground=self.node.create_publisher(Empty,'/flyco/ground/heartbeat',1)
        self.command=self.node.create_publisher(PositionCommand,'/planning/pos_cmd',10)
        self.ack=self.node.create_publisher(VehicleCommandAck,'/fmu/out/vehicle_command_ack',qos_profile_sensor_data)
        self.registration_reply=self.node.create_publisher(RegisterExtComponentReply,
            topic('/fmu','out','register_ext_component_reply',RegisterExtComponentReply),qos_profile_sensor_data)
        self.node.create_subscription(RegisterExtComponentRequest,
            topic('/fmu','in','register_ext_component_request',RegisterExtComponentRequest),
            self.on_registration,qos_profile_sensor_data)
        self.completed=self.node.create_publisher(ModeCompleted,'/fmu/out/mode_completed',qos_profile_sensor_data)
        self.node.create_subscription(VehicleCommand,'/fmu/in/vehicle_command_mode_executor',
                                     self.on_request,qos_profile_sensor_data)
        self.node.create_subscription(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',self.targets.append,qos_profile_sensor_data)
        self.node.create_subscription(VehicleControlMode,'/fmu/in/config_control_setpoints',self.heartbeats.append,qos_profile_sensor_data)
        self.node.create_subscription(VehicleCommand,'/fmu/in/vehicle_command',self.on_request,qos_profile_sensor_data)
        self.timer=self.node.create_timer(.02,self.publish)

    def on_registration(self,msg):
        assert bytes(msg.name).rstrip(b'\0') == b'FlyCo'
        assert msg.register_mode and msg.register_arming_check and not msg.enable_replace_internal_mode
        self.registration_reply.publish(RegisterExtComponentReply(
            timestamp=self.node.get_clock().now().nanoseconds//1000, request_id=msg.request_id,
            name=msg.name, success=True, arming_check_id=0, mode_id=23, mode_executor_id=1,
            px4_ros2_api_version=msg.px4_ros2_api_version))

    def on_request(self,msg):
        self.requests.append(msg)
        if msg.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
            self.live['mode'] = VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
        elif msg.command == VehicleCommand.VEHICLE_CMD_SET_NAV_STATE:
            self.live['mode'] = int(msg.param1)
        if self.live['ack'] is not None:
            ack=VehicleCommandAck(timestamp=self.node.get_clock().now().nanoseconds//1000,command=msg.command,
                                  target_system=msg.source_system,target_component=msg.source_component,
                                  result=self.live['ack'])
            if self.live['wrong_ack_target']: ack.target_component+=1
            if self.live['ack_future']: ack.timestamp+=10000000
            self.ack.publish(ack)

    def publish(self):
        stamp=self.node.get_clock().now().to_msg()
        us=self.node.get_clock().now().nanoseconds//1000+int(self.live['clock_offset']*1.e6)
        if self.live['odom']:
            msg=Odometry(); msg.header.stamp=stamp; msg.header.frame_id='world'; msg.child_frame_id='base_link'
            msg.pose.pose.position.z=1.; msg.pose.pose.orientation.w=1.; self.odom.publish(msg)
        if self.live['local']:
            self.local.publish(VehicleLocalPosition(timestamp=us,timestamp_sample=us,xy_valid=self.live['valid'],
                z_valid=True,heading_good_for_control=True,x=0.,y=10.,z=-2.,heading=0.,xy_reset_counter=self.live['reset']))
        if self.live['state']:
            self.state.publish(VehicleStatus(timestamp=us,system_id=1,component_id=1,executor_in_charge=1,nav_state=self.live['mode'],failsafe=self.live['failsafe'],
                arming_state=VehicleStatus.ARMING_STATE_ARMED if self.live['armed'] else VehicleStatus.ARMING_STATE_DISARMED))
        if self.live['ground']: self.ground.publish(Empty())
        if self.live['command'] is not None:
            msg=PositionCommand();msg.header.stamp=stamp;msg.header.frame_id='world'
            msg.position.x=self.live['command'];msg.position.z=1.
            msg.velocity.x=.2;msg.acceleration.x=.1;msg.yaw=.1;msg.yaw_dot=.2;msg.trajectory_flag=1
            self.command.publish(msg)

    def spin(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline: self.executor.spin_once(timeout_sec=.005)

    def service(self,name,kind=Trigger,value=None):
        client=self.node.create_client(kind,'/flyco/control/'+name)
        assert client.wait_for_service(timeout_sec=2.)
        request=kind.Request() if value is None else kind.Request(data=value)
        future=client.call_async(request); deadline=time.monotonic()+2.
        while not future.done() and time.monotonic()<deadline: self.executor.spin_once(timeout_sec=.01)
        result=future.result(); self.node.destroy_client(client)
        assert result is not None
        return result

    def engage(self):
        self.spin(.5)
        assert self.service('enable',SetBool,True).success
        assert not self.service('resume').success
        self.spin(1.25)
        assert self.bridge.flight_mode.mode_id == 23
        self.live.update(armed=True,mode=23)
        self.spin(.1)
        assert self.service('resume').success

    def close(self):
        for node in (self.node,self.bridge): self.executor.remove_node(node); node.destroy_node()
        self.executor.shutdown()


@pytest.fixture
def dds():
    rclpy.init(); fixture=DDSFixture()
    try: yield fixture
    finally: fixture.close();rclpy.shutdown()


def test_dds_setpoints_gating_and_watchdogs(dds):
    dds.spin(.6)
    assert dds.targets==dds.requests==[]
    statuses=[]
    dds.node.create_subscription(String,'/flyco/control/status',statuses.append,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    dds.spin(.2)
    state=json.loads(statuses[-1].data)
    assert state['odom_ok'] and state['px4_ok'] and state['ground_ok'] and not state['enabled']
    dds.engage()
    assert np.isnan(dds.targets[-1].velocity).all()
    dds.live['command']=.5;dds.spin(.2)
    cmd=dds.targets[-1]
    assert np.allclose(cmd.position,[.5,10.,-2.],atol=1.e-5)
    assert np.allclose(cmd.velocity,[.2,0.,0.],atol=1.e-5)
    assert np.allclose(cmd.acceleration,[.1,0.,0.],atol=1.e-5)
    assert cmd.yaw==pytest.approx(-.1) and cmd.yawspeed==pytest.approx(-.2)
    assert dds.heartbeats[-1].flag_control_position_enabled
    assert not dds.heartbeats[-1].flag_control_offboard_enabled
    assert dds.heartbeats[-1].source_id == 23
    assert not dds.requests  # No automatic arming or mode switching.
    dds.live['command']=None;dds.spin(.45)
    assert not dds.bridge.tracking and 'timeout' in dds.bridge.reason
    assert np.isnan(dds.targets[-1].velocity).all()
    dds.live['command']=.5;dds.spin(.1);assert not dds.bridge.tracking
    assert dds.service('resume').success
    dds.live['command']=5.;dds.spin(.1)
    assert not dds.bridge.tracking and 'tracking error' in dds.bridge.reason
    dds.live['command']=.5;assert dds.service('resume').success
    dds.bridge.settings['ground_link_timeout']=.15
    dds.live['ground']=False;dds.spin(.25)
    assert not dds.bridge.tracking and 'ground link' in dds.bridge.reason
    dds.live.update(ground=True,mode=VehicleStatus.NAVIGATION_STATE_POSCTL)
    dds.spin(.1)
    assert not dds.bridge.enabled
    count=len(dds.targets);dds.spin(.15)
    assert count==len(dds.targets)


def test_native_executor_takeoff_returns_to_registered_mode(dds):
    dds.engage()
    assert dds.service('takeoff').success
    dds.spin(.2)
    cmd = next(c for c in dds.requests if c.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
    assert cmd.source_component == VehicleCommand.COMPONENT_MODE_EXECUTOR_START+1
    assert math.isnan(cmd.param5) and math.isnan(cmd.param6)
    assert cmd.param7 == pytest.approx(4.)
    assert dds.bridge.enabled and dds.bridge.takeoff_phase == 'climbing'
    dds.completed.publish(ModeCompleted(nav_state=VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF,
                                       result=ModeCompleted.RESULT_SUCCESS))
    dds.spin(.3)
    assert dds.bridge.takeoff_phase == 'complete'
    assert dds.bridge.armed_flyco() and dds.bridge.endpoint_wait
    assert dds.bridge.hold[0][2] == pytest.approx(-4.)
    assert not any(c.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM for c in dds.requests)


def test_auto_enable_waits_for_ready_and_reengages_only_after_position_switch(dds):
    dds.bridge.settings['auto_enable'] = True
    dds.bridge.auto_enable_pending = True
    dds.live['ground'] = False
    dds.spin(.5)
    assert not dds.bridge.enabled
    dds.live['ground'] = True
    dds.spin(1.4)
    assert dds.bridge.enabled and not dds.requests
    dds.live['mode'] = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
    dds.spin(.1)
    dds.live['mode'] = VehicleStatus.NAVIGATION_STATE_POSCTL
    dds.spin(.1)
    assert not dds.bridge.auto_enable_pending
    dds.live.update(armed=True, mode=23)
    dds.spin(.1)
    dds.live['valid'] = False
    dds.spin(.1)
    dds.live['valid'] = True
    dds.spin(1.4)
    assert not dds.bridge.enabled  # A fault does not recover silently in FlyCo.
    dds.live['mode'] = VehicleStatus.NAVIGATION_STATE_POSCTL
    dds.spin(1.4)
    assert dds.bridge.enabled and not dds.requests
    assert dds.service('enable', SetBool, False).success
    dds.live['mode'] = 23
    dds.spin(.1)
    dds.live['mode'] = VehicleStatus.NAVIGATION_STATE_POSCTL
    dds.spin(1.4)
    assert not dds.bridge.enabled  # Explicit disable stays disabled.


def test_small_explicit_heading_reset_reanchors_without_disabling(dds):
    import copy
    dds.engage()
    before = copy.deepcopy(dds.bridge.local)
    msg = copy.deepcopy(before)
    msg.timestamp += 1000
    msg.heading_reset_counter += 1
    msg.delta_heading = .0055
    msg.heading += .0055
    dds.bridge.on_local(msg)
    assert dds.bridge.enabled
    assert 'heading-only reset compensated' in dds.bridge.reason
    rotation, offset, yaw = dds.bridge.alignment
    world = np.array([0.,0.,1.])
    from flyco_real.px4_dds import local_pose_enu
    np.testing.assert_allclose(rotation @ world+offset, local_pose_enu(msg)[:3,3], atol=1.e-6)


def test_large_heading_reset_still_stops_control(dds):
    import copy
    dds.engage()
    msg = copy.deepcopy(dds.bridge.local)
    msg.timestamp += 1000
    msg.heading_reset_counter += 1
    msg.delta_heading = 1.
    msg.heading += 1.
    dds.bridge.on_local(msg)
    assert not dds.bridge.enabled


@pytest.mark.parametrize('failure',['invalid_position','clock','ekf_reset','odom_lost','failsafe'])
def test_dds_invalid_estimator_or_link_stops_both_streams(dds,failure):
    dds.engage()
    if failure=='invalid_position': dds.live['valid']=False
    elif failure=='clock': dds.live['clock_offset']=-10.
    elif failure=='ekf_reset': dds.live['reset']=1
    elif failure=='failsafe': dds.live['failsafe']=True
    else: dds.live['odom']=False
    dds.spin(.5)
    assert not dds.bridge.enabled
    count=len(dds.targets);dds.spin(.1)
    assert count==len(dds.targets)
    if failure=='ekf_reset': assert 'estimator reset' in dds.bridge.reason


@pytest.mark.parametrize('result,wrong_target,future,expected',[(0,False,False,'accepted'),(2,False,False,'rejected'),(0,True,False,'ack timeout'),(None,False,False,'ack timeout'),(0,False,True,'ack timeout')])
def test_dds_landing_acknowledgment(dds,result,wrong_target,future,expected):
    dds.spin(.5)
    dds.bridge.settings['command_ack_timeout']=.15
    dds.live.update(ack=result,wrong_ack_target=wrong_target,ack_future=future)
    assert dds.service('land').success
    dds.spin(.25)
    assert len(dds.requests)==1
    command=dds.requests[0]
    assert command.command==VehicleCommand.VEHICLE_CMD_NAV_LAND
    assert command.target_system==1 and command.source_component==191 and command.from_external
    assert math.isnan(command.param5) and math.isnan(command.param6)
    assert dds.bridge.land_result.startswith(expected)
    assert not dds.targets
