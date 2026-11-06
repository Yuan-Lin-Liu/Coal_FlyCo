"""Transport boundary tests; actual PX4/Isaac flight is verified separately."""
import importlib.util
from pathlib import Path
import sys
import types

import numpy as np
import pytest


@pytest.fixture
def backend(monkeypatch):
    class Connection:
        address = ('127.0.0.1', 14580)
        def __init__(self):
            self.messages, self.calls = [], []
            self.mav = self
        def recv_match(self, **kwargs):
            return self.messages.pop(0) if self.messages else None
        def set_position_target_local_ned_send(self, *args):
            self.calls.append(('target', args))
        def heartbeat_send(self, *args):
            self.calls.append(('heartbeat', args))
        def command_long_send(self, *args):
            self.calls.append(('command', args))
    connection = Connection()
    constants = types.SimpleNamespace(MAV_MODE_FLAG_SAFETY_ARMED=128,
        MAV_TYPE_ONBOARD_CONTROLLER=18, MAV_AUTOPILOT_INVALID=8,
        MAV_FRAME_LOCAL_NED=1, MAV_CMD_DO_SET_MODE=176,
        MAV_CMD_COMPONENT_ARM_DISARM=400, MAV_CMD_NAV_LAND=21)
    mavutil = types.SimpleNamespace(mavlink=constants,
        mavlink_connection=lambda *a, **k: connection)
    monkeypatch.setitem(sys.modules, 'pymavlink', types.SimpleNamespace(mavutil=mavutil))
    class Parent:
        def __init__(self, *args): pass
        def update(self, dt): pass
        def input_reference(self): return [0., 0., 0., 0.]
    module = types.ModuleType('pegasus.simulator.logic.backends.px4_mavlink_backend')
    module.PX4MavlinkBackend = Parent
    module.PX4MavlinkBackendConfig = lambda value: value
    monkeypatch.setitem(sys.modules, module.__name__, module)
    name = 'warehouse_px4_test'
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[1]/'sim/px4_backend.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    b = module.WarehousePX4Backend()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.)
    b.state = types.SimpleNamespace(position=np.array([-14., 0., 2.]),
        linear_velocity=np.zeros(3), attitude=[0.,0.,0.,1.])
    b.offset = np.array([0.,-14.,0.])
    b.initialized = True
    b.heartbeat_wall = 100.
    return b, connection


def command(**values):
    return dict(p=[-10.,2.,3.], v=[1.,2.,3.], a=[.1,.2,.3],
        yaw=0., yaw_dot=.4, count=1, **values)


def test_disabled_never_sends_setpoints_or_arms(backend):
    b, link = backend
    b.set_command(command())
    b.update(.004)
    assert not any(k in ('target','command') for k,_ in link.calls)


@pytest.mark.parametrize('native',[False,True])
def test_warehouse_native_rtl_requests_land_once_without_arming(backend,native):
    b,link=backend
    b.native_mode=native
    message=types.SimpleNamespace(base_mode=128,custom_mode=(4<<16)|(5<<24),
        get_srcSystem=lambda:1,get_srcComponent=lambda:1,get_type=lambda:'HEARTBEAT')
    link.messages.append(message)
    b.update(.004)
    link.messages.append(message)
    b.update(.004)
    calls=[args for kind,args in link.calls if kind=='command']
    if native:
        assert len(calls)==1 and calls[0][2]==21
        assert b.landing_state=='requested'
        assert b.telemetry()['last_safety_action']['action']=='Land'
    else:
        assert calls==[] and b.px4_mode=='RTL'
    assert not any(args[2]==400 for args in calls)


def test_enu_world_origin_and_yaw_conversion(backend):
    b, link = backend
    b.set_command(command())
    b.active = b.tracking = True
    b.update(.004)
    args = next(v for k,v in link.calls if k=='target')
    assert args[4] == 0  # position+velocity+acceleration+yaw+yaw-rate enabled
    np.testing.assert_allclose(args[5:8], [2.,4.,-3.])
    np.testing.assert_allclose(args[8:11], [2.,1.,-3.])
    np.testing.assert_allclose(args[11:14], [.2,.1,-.3])
    np.testing.assert_allclose(args[14:], [np.pi/2,-.4])


def test_qgc_mode_takeover_stops_stream_and_never_rearms(backend):
    b, link = backend
    b.active = b.tracking = True
    b.px4_mode = 'OFFBOARD'
    message = types.SimpleNamespace(base_mode=128, custom_mode=(4<<16)|(6<<24),
        get_srcSystem=lambda:1,get_srcComponent=lambda:1,get_type=lambda:'HEARTBEAT')
    link.messages.append(message)
    b.update(.004)
    assert b.px4_mode == 'LAND'
    assert not b.active and not b.tracking
    assert not any(k in ('target','command') for k,_ in link.calls)


def test_completed_trajectory_keeps_endpoint(backend):
    b, link = backend
    c=command();c.update(v=[0.,0.,0.],a=[0.,0.,0.],p=[-14.,0.,2.])
    b.set_command(c)
    b.command_wall=98.
    b.active=b.tracking=True
    b.update(.004)
    assert not b.tracking and b.fault==''
    np.testing.assert_allclose(b.target,[-14.,0.,2.])


def test_midflight_stale_command_holds_current_position(backend):
    b, link = backend
    b.set_command(command());b.command_wall=98.
    b.active=b.tracking=True
    b.update(.004)
    np.testing.assert_allclose(b.target,b.state.position)
    assert 'stale' in b.fault


def test_first_trajectory_has_generation_grace(backend):
    b, link = backend
    b.active=b.tracking=True
    b.command_wall=98.;b.capture_hold()
    b.update(.004)
    assert b.tracking and not b.fault


def test_nonfinite_command_is_rejected(backend):
    b, link = backend
    c=command();c['p'][0]=float('nan')
    b.set_command(c)
    assert b.command is None and b.command_count==0


def test_missing_px4_heartbeat_stops_offboard(backend):
    b, link = backend
    b.active=b.tracking=True;b.heartbeat_wall=95.
    b.update(.004)
    assert not b.active and not b.tracking
    assert not any(k=='target' for k,_ in link.calls)


def message(kind, **fields):
    return types.SimpleNamespace(get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1, get_type=lambda: kind, **fields)


def finish_segment(b):
    c = command()
    c.update(v=[0.,0.,0.], a=[0.,0.,0.], p=[-14.,0.,2.],
             trajectory_end=10., t=9.99)
    b.set_command(c)
    b.command_wall = 98.
    b.active = b.tracking = b.accept_trajectories = b.armed = True
    b.px4_mode = 'OFFBOARD'
    b.update(.004)


def test_next_scan_segment_resumes_after_normal_endpoint(backend):
    b, link = backend
    finish_segment(b)
    assert b.endpoint_wait and not b.tracking and not b.fault
    c = command(); c['count'] = 2
    b.set_command(c)
    b.last_send = 0.
    link.calls.clear()
    b.update(.004)
    assert b.tracking and not b.endpoint_wait
    target = next(v for k,v in link.calls if k == 'target')
    np.testing.assert_allclose(target[5:8], [2.,4.,-3.])


def test_hold_does_not_resume_when_new_scan_commands_arrive(backend):
    b, link = backend
    finish_segment(b)
    b.action('hold')
    c = command(); c['count'] = 2
    b.set_command(c)
    assert not b.tracking and not b.accept_trajectories
    np.testing.assert_allclose(b.target, b.state.position)


def test_stationary_midtrajectory_loss_requires_resume(backend):
    b, link = backend
    c = command()
    c.update(v=[0.,0.,0.], a=[0.,0.,0.], trajectory_end=10., t=3.)
    b.set_command(c); b.command_wall = 98.
    b.active = b.tracking = b.accept_trajectories = True
    b.update(.004)
    assert 'stale' in b.fault and not b.accept_trajectories


def test_qgc_takeover_can_be_explicitly_resumed_with_new_warmup(backend):
    b, link = backend
    b.active = b.tracking = True
    b.px4_mode = 'OFFBOARD'
    link.messages.append(message('HEARTBEAT', base_mode=128,
                                 custom_mode=(4<<16)|(3<<24)))
    b.update(.004)
    assert not b.active
    b.action('resume')
    assert b.active and b.pending == 'warmup' and b.stream_started == 100.
    link.calls.clear()
    b.update(.004)
    assert not any(k == 'command' for k,_ in link.calls)
    b.stream_started = 98.
    b.update(.004)
    assert b.pending == 'wait_mode'
    assert any(k == 'command' and v[2] == 176 for k,v in link.calls)


def test_qgc_ack_does_not_cancel_our_pending_command(backend):
    b, link = backend
    b.capture_hold()
    b.active = True
    b.pending = 'wait_mode'
    b.command_long(176, 1., 6.)
    link.messages.append(message('COMMAND_ACK', command=176, result=2,
                                 target_system=255, target_component=190))
    b.update(.004)
    assert b.pending == 'wait_mode' and not b.fault and b.ack is None


def test_own_rejected_arm_stops_automatic_requests(backend):
    b, link = backend
    b.capture_hold()
    b.active = True
    b.pending = 'wait_arm'
    b.command_long(400, 1.)
    link.messages.append(message('COMMAND_ACK', command=400, result=2,
                                 target_system=245, target_component=191))
    b.update(.004)
    assert b.pending is None and 'rejected' in b.fault
    assert not b.tracking


def test_land_preserves_hold_until_confirmed_then_waits_for_stopped_motors(backend):
    b, link = backend
    b.active = b.armed = b.tracking = b.accept_trajectories = True
    b.px4_mode = 'OFFBOARD'
    b.action('land')
    args = next(v for k,v in link.calls if k == 'command')
    assert args[2] == 21 and np.all(np.isnan(args[7:11]))
    assert b.active and not b.accept_trajectories and b.pending == 'land'
    link.messages.append(message('HEARTBEAT', base_mode=128,
                                 custom_mode=(4<<16)|(6<<24)))
    b.update(.004)
    assert not b.active and b.landing_state == 'descending'
    assert not b.telemetry()['landing_complete']
    b.state.position[2] = .062
    link.messages.append(message('HEARTBEAT', base_mode=0,
                                 custom_mode=(4<<16)|(6<<24)))
    b.update(.004)
    assert b.telemetry()['landing_complete'] and not b.fault


def test_land_confirmation_retries_are_bounded(backend):
    b, link = backend
    b.active = b.armed = True
    b.action('land')
    for _ in range(3):
        b.pending_since = 96.
        b.update(.004)
    assert b.landing_attempts == 3 and b.landing_state == 'failed'
    assert b.pending is None and 'not confirmed' in b.fault


def test_above_ground_disarm_does_not_disable_the_hold_stream(backend):
    b, link = backend
    b.active = b.tracking = True
    b.action('disarm')
    assert b.active and b.tracking and 'rejected' in b.fault
    assert not any(k == 'command' for k,_ in link.calls)


def test_pending_arm_is_cancelled_if_qgc_changes_mode(backend):
    b, link = backend
    b.active = True
    b.px4_mode = 'OFFBOARD'
    b.pending = 'wait_arm'
    link.messages.append(message('HEARTBEAT', base_mode=0,
                                 custom_mode=(4<<16)|(3<<24)))
    b.update(.004)
    assert b.pending is None and not b.active
    assert not any(k == 'command' for k,_ in link.calls)
