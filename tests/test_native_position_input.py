"""Position arming receives neutral input only in the native SITL profile."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.mark.parametrize('profile,expected', [('px4-native', '8'), ('px4', '4')])
def test_sitl_manual_input_profile_does_not_change_legacy_or_real_parameters(tmp_path, monkeypatch, profile, expected):
    runtime = module('position_qgc_runtime', ROOT/'tools/qgc_runtime.py')
    calls = []
    monkeypatch.setattr(runtime.subprocess, 'Popen', lambda *args, **kwargs: calls.append(kwargs))
    run = tmp_path/'run'
    run.mkdir()
    runtime.start_px4(tmp_path/'firmware', run, dict(FLYCO_ISAAC_FLIGHT_CONTROLLER=profile, FLYCO_ISAAC_DOMAIN='68'))
    assert calls[0]['env']['PX4_PARAM_COM_RC_IN_MODE'] == expected
    assert calls[0]['cwd'] == run/'px4'
    assert not (tmp_path/'firmware').exists()


def test_virtual_sticks_stop_when_the_px4_clock_stream_is_stale(monkeypatch):
    sticks = module('position_sim_sticks', ROOT/'bridge/sim_manual.py')
    clock, sent = [10.], []
    monkeypatch.setattr(sticks.time, 'monotonic', lambda: clock[0])
    node = SimpleNamespace(create_publisher=lambda *a: SimpleNamespace(publish=sent.append),
        create_subscription=lambda *a: None, create_timer=lambda *a: None)
    source = sticks.NeutralSimSticks(node)
    source.publish()
    assert not sent
    source.on_position(SimpleNamespace(timestamp=123456))
    source.publish()
    assert len(sent) == 1 and sent[0].valid
    assert sent[0].data_source == sticks.ManualControlSetpoint.SOURCE_MAVLINK_5
    assert sent[0].timestamp_sample == 123456 and sent[0].throttle == 0.
    assert sent[0].roll == sent[0].pitch == sent[0].yaw == 0.
    clock[0] += .31
    source.publish()
    assert len(sent) == 1
