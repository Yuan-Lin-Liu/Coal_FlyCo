"""Keep the simulator's QGC locale and files separate from operator settings."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace


def test_qgc_uses_session_configuration_and_clean_qt_environment(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('qgc_runtime_test',
        Path(__file__).resolve().parents[1]/'tools/qgc_runtime.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.subprocess, 'run', lambda *a, **kw: SimpleNamespace(returncode=1))
    calls = []
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *a, **kw: calls.append((a,kw)))
    executable = tmp_path/'QGroundControl'
    executable.touch(); executable.chmod(0o755)
    run = tmp_path/'session'; run.mkdir()
    module.start_qgc(executable, run, {'DISPLAY': ':1', 'QT_PLUGIN_PATH': '/ros/qt',
        'LD_LIBRARY_PATH': '/ros/lib', 'XAUTHORITY': '/desktop/auth'})
    env = calls[0][1]['env']
    assert env['LANG'] == env['LC_ALL'] == 'en_US.UTF-8'
    assert env['XAUTHORITY'] == '/desktop/auth'
    assert 'QT_PLUGIN_PATH' not in env and 'LD_LIBRARY_PATH' not in env
    config = Path(env['XDG_CONFIG_HOME'])/'QGroundControl/QGroundControl.ini'
    assert 'qLocaleLanguage=0' in config.read_text()
    assert str(run/'qgc-data') in config.read_text()
    assert env['XDG_DATA_HOME'] == str(run/'qgc-data')
