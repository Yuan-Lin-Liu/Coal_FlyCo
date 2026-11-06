"""Host PX4 SITL/QGC processes, isolated from real-flight workspaces."""
import os
import json
from pathlib import Path
import socket
import subprocess


def preflight(px4_dir):
    root = Path(px4_dir).expanduser().resolve()
    for path in (root/'build/px4_sitl_default/bin/px4',
                 root/'ROMFS/px4fmu_common/init.d-posix/rcS'):
        if not path.is_file():
            raise FileNotFoundError(f'PX4 SITL missing: {path}. See docs/px4_qgc.md')
    # Never silently connect this simulator to an existing PX4/vehicle session.
    for port, kind in ((4560, socket.SOCK_STREAM), (14540, socket.SOCK_DGRAM),
                       (14580, socket.SOCK_DGRAM), (18570, socket.SOCK_DGRAM)):
        with socket.socket(socket.AF_INET, kind) as probe:
            try:
                probe.bind(('127.0.0.1', port))
            except OSError as exc:
                raise RuntimeError(f'PX4 simulation port {port} is occupied; stop the other SITL session') from exc
    return root


def start_px4(root, run, env):
    work = run/'px4'
    work.mkdir()
    # Only this fresh SITL filesystem is configured. Real PX4 and QGC settings
    # are untouched. Simulated GPS/barometer/magnetometer feed the PX4 EKF.
    px4_env = dict(env, PX4_SIM_MODEL='gazebo-classic_iris',
        PX4_SYS_AUTOSTART='10015', PX4_PARAM_COM_RC_IN_MODE='4')
    if env.get('FLYCO_ISAAC_FLIGHT_CONTROLLER') == 'px4-native':
        px4_env.update(PX4_PARAM_UXRCE_DDS_DOM_ID=env['FLYCO_ISAAC_DOMAIN'],
                       PX4_PARAM_UXRCE_DDS_SYNCT='0', ROS_DOMAIN_ID=env['FLYCO_ISAAC_DOMAIN'],
                       PX4_PARAM_COM_RC_IN_MODE='8')
    version = root/'build/px4_sitl_default/src/lib/version/build_git_version.h'
    (run/'px4_build.json').write_text(json.dumps({
        'root': str(root), 'binary': str(root/'build/px4_sitl_default/bin/px4'),
        'build_version': version.read_text() if version.is_file() else None,
        'model': px4_env['PX4_SIM_MODEL'], 'autostart': 10015}, indent=2)+'\n')
    log = (run/'px4.log').open('wb')
    try:
        return subprocess.Popen([str(root/'build/px4_sitl_default/bin/px4'),
            str(root/'ROMFS/px4fmu_common'), '-s',
            str(root/'ROMFS/px4fmu_common/init.d-posix/rcS'), '-i', '0', '-d'],
            cwd=work, env=px4_env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
    finally:
        log.close()


def start_qgc(executable, run, env):
    existing = subprocess.run(['pgrep', '-u', str(os.getuid()), '-x', 'QGroundControl'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if existing.returncode == 0:
        print('QGC 已在运行，复用现有地面站；若出现未知中文飞行模式，请退出 QGC 后用 --qgc 重新启动。', flush=True)
        return None
    path = Path(executable).expanduser().resolve()
    if not path.is_file() or not os.access(path, os.X_OK):
        raise FileNotFoundError(f'QGroundControl executable not found: {path}; use --qgc-path')
    if not env.get('DISPLAY') and not env.get('WAYLAND_DISPLAY'):
        raise RuntimeError('--qgc requires a graphical desktop; open QGC separately for headless simulation')
    # A fresh session config avoids the installed QGC's translated mode-name
    # mismatch without modifying the operator's normal ground-station settings.
    config = run/'qgc-config'
    settings = config/'QGroundControl/QGroundControl.ini'
    settings.parent.mkdir(parents=True, exist_ok=True)
    data = run/'qgc-data'
    data.mkdir(exist_ok=True)
    settings.write_text('[General]\nqLocaleLanguage=0\nfirstRunPromptIdsShown="1,2"\n'
                        f'savePath={data}\n\n[AutoConnect]\nUDP=true\n')
    # Also isolate caches and remove ROS/OpenCV's library and Qt plugin paths.
    qgc_env = {k: v for k, v in env.items()
               if not k.startswith(('QT_', 'QML', 'LD_LIBRARY_PATH', 'LD_PRELOAD'))}
    qgc_env.update(XDG_CONFIG_HOME=str(config), XDG_DATA_HOME=str(data),
                   XDG_CACHE_HOME=str(run/'qgc-cache'),
                   LANG='en_US.UTF-8', LC_ALL='en_US.UTF-8', LANGUAGE='en_US')
    log = (run/'qgc.log').open('wb')
    try:
        return subprocess.Popen([str(path)], cwd=run, env=qgc_env,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    finally:
        log.close()
