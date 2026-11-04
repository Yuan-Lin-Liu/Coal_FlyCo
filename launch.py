#!/usr/bin/env python3
"""Manage one isolated FlyCo_ROS2 + Isaac session and its own child processes."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid
import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'bridge'))
sys.path.insert(0, str(ROOT/'tools'))
import container_runtime
from console_output import ConsoleOutput
from scene_profiles import resolve_scene

RUNTIME = ROOT/'runtime'


def current():
    return (RUNTIME/'latest').resolve(strict=True)


def process_alive(info):
    try:
        fields = Path('/proc', str(info['pid']), 'stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] == info['birth'] and fields[0] != 'Z'
    except (OSError, KeyError):
        return False


def process_info(pid):
    return {'pid': pid, 'birth': Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()[19]}


def startup_failure(run, returncode, timed_out=False):
    """Keep the original failure cause instead of reporting every exit as init failure."""
    log = run/'isaac.log'
    output = log.read_text(errors='replace') if log.exists() else ''
    killed = returncode == -signal.SIGKILL or bool(re.search(
        r'^.*:\s*line\s+\d+:\s+\d+\s+Killed\b', output, re.MULTILINE))
    if killed:
        reason = 'process_killed'
        message = ('Isaac Python was forcibly killed (SIGKILL). '
                   'This alone does not prove an out-of-memory condition.')
    elif timed_out:
        reason = 'startup_timeout'
        message = 'Isaac initialization exceeded the 120-second timeout.'
    else:
        reason = 'process_exited'
        message = f'Isaac exited during initialization (launcher exit code {returncode}).'
    diagnostic = {'reason': reason, 'launcher_exit_code': returncode,
                  'message': message, 'log_path': str(log),
                  'log_tail': output.splitlines()[-25:]}
    (run/'startup_failure.json').write_text(json.dumps(diagnostic, indent=2)+'\n')
    return diagnostic


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', nargs='?', default='run', choices=('run', 'start', 'stop', 'status', 'action', 'logs', 'rviz'))
    parser.add_argument('action', nargs='?', choices=('initialize', 'initial', 'plan', 'scan', 'hold', 'land', 'resume', 'disarm'))
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--docker', action='store_true', help='Run Isaac and FlyCo in project Docker images')
    parser.add_argument('--no-perception', action='store_true')
    parser.add_argument('--segmentation-backend', type=lambda value: 'semantic' if value.lower() == 'a' else value.lower(),
                        choices=('semantic','native'), default='semantic',
                        help='Semantic Perception (default) or original EVF-SAM2/SAMURAI')
    parser.add_argument('--segmentation-endpoint', default='http://127.0.0.1:8851')
    parser.add_argument('--no-ui', action='store_true')
    visualization = parser.add_mutually_exclusive_group()
    visualization.add_argument('--rviz', dest='rviz', action='store_true',
                               help='Open RViz (default for graphical sessions)')
    visualization.add_argument('--no-rviz', dest='rviz', action='store_false',
                               help='Do not open RViz automatically')
    parser.set_defaults(rviz=None)
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--scene', '--pile', default='4',
                        help='Single-pile scene: 4 (Final, default), 6 (Solid), or a custom USD path')
    parser.add_argument('--domain', type=int, default=64)
    parser.add_argument('--port', type=int, default=8870)
    parser.add_argument('--verbose', action='store_true', help='Mirror complete Isaac/ROS output to the terminal')
    parser.add_argument('--once', action='store_true', help='Print saved logs without following (logs command)')
    args = parser.parse_args()
    RUNTIME.mkdir(exist_ok=True)
    if args.command in ('status', 'stop', 'action', 'logs', 'rviz'):
        run = current()
        state = json.loads((run/'session.json').read_text())
        if args.command == 'rviz':
            if 'ended' in state or not process_alive(state['manager']):
                raise RuntimeError('No running session. Start ./launch_warehouse.sh first.')
            env = dict(os.environ, FLYCO_ISAAC_DOMAIN=str(state['domain']), FLYCO_ISAAC_RUNTIME=str(run))
            print(f'打开当前会话的 RViz（ROS_DOMAIN_ID={state["domain"]}）。关闭 RViz 不会停止仿真。', flush=True)
            command = ['bash', str(ROOT/'tools/run_rviz.sh')]
            if state.get('docker'):
                command = container_runtime.command('flyco', command, run, env, suffix='-rviz')
            child = subprocess.Popen(command, env=env,
                                     cwd=ROOT, start_new_session=True)
            try:
                while child.poll() is None:
                    if (run/'stop.request').exists() or 'ended' in json.loads((run/'session.json').read_text()):
                        break
                    if not process_alive(state['manager']):
                        break
                    time.sleep(.3)
            except KeyboardInterrupt:
                pass
            finally:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.terminate()
                        child.wait(timeout=5)
            return child.returncode if child.returncode and child.returncode > 0 else 0
        elif args.command == 'logs':
            output = ConsoleOutput(run, verbose=args.verbose)
            print('会话日志：'+str(run), flush=True)
            try:
                while True:
                    output.emit()
                    if args.once or 'ended' in json.loads((run/'session.json').read_text()):
                        break
                    time.sleep(.25)
            except KeyboardInterrupt:
                print('已停止查看日志；仿真继续运行。', flush=True)
        elif args.command == 'status':
            print(json.dumps(dict(state, alive=process_alive(state['manager']),
                run_dir=str(run), status=json.loads((run/'sim_status.json').read_text())
                if (run/'sim_status.json').exists() else None), indent=2))
        elif args.command == 'stop':
            (run/'stop.request').touch()
            print('Requested normal shutdown:', run)
        else:
            if not args.action:
                parser.error('action requires an action name')
            if not process_alive(state['manager']):
                raise RuntimeError('Session is no longer running')
            (run/'actions'/f'{time.time_ns()}-{uuid.uuid4().hex}.json').write_text(
                json.dumps({'action': args.action}))
            print('Queued:', args.action)
        return 0
    try:
        profile = resolve_scene(args.scene)
    except ValueError as exc:
        parser.error(str(exc))
    if not profile['scene'].is_file():
        raise FileNotFoundError('Scene is missing: '+str(profile['scene'])+
            '. Run ./isaac_python.sh tools/rebuild_scene.py --scene '+profile['id'])
    if args.command == 'start':
        log = (RUNTIME/'launcher.log').open('ab')
        argv = [sys.executable, str(ROOT/'launch.py'), 'run']+sys.argv[2:]
        child = subprocess.Popen(argv, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        print('Launcher started:', child.pid, 'log:', RUNTIME/'launcher.log')
        return 0
    flyco_runtime = ROOT/'.local/flyco'
    if args.docker:
        container_runtime.preflight()
    if not args.docker and not (flyco_runtime/'interfaces/install/local_setup.bash').is_file():
        raise FileNotFoundError('Build project-local FlyCo first: ./tools/setup_flyco.sh')
    for binary in ('flyco_planner', 'traj_server', 'px4ctrl_node', 'joy_cmd',
                   'rgb_point_mapping_node', 'sam_tracking_node', 'kdline'):
        if not args.docker and not (flyco_runtime/'build'/binary).is_file():
            raise FileNotFoundError('Run ./tools/setup_flyco.sh first; missing '+binary)
    lock = (RUNTIME/'session.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('已有仿真会话正在运行。请先执行 python3 launch.py stop，等待退出后再启动所选场景。')
    run = RUNTIME/('run-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
    run.mkdir()
    (run/'actions').mkdir()
    (run/'scene_config.yaml').write_text(yaml.safe_dump(profile['config'], sort_keys=False))
    (run/'initial_waypts.yaml').write_text(profile['waypoints'].read_text())
    latest = RUNTIME/'latest'
    latest.unlink(missing_ok=True)
    latest.symlink_to(run.name, target_is_directory=True)
    rviz_enabled = args.rviz if args.rviz is not None else not (args.headless or args.no_ui)
    env = dict(os.environ, FLYCO_ISAAC_RUNTIME=str(run), FLYCO_ISAAC_PORT=str(args.port),
        FLYCO_ISAAC_DOMAIN=str(args.domain), FLYCO_ISAAC_NO_PERCEPTION=str(int(args.no_perception)),
        FLYCO_ISAAC_NO_UI=str(int(args.no_ui or args.headless)), FLYCO_ISAAC_RVIZ=str(int(rviz_enabled)),
        FLYCO_ISAAC_SEGMENTATION_BACKEND=args.segmentation_backend,
        FLYCO_ISAAC_SEGMENTATION_ENDPOINT=args.segmentation_endpoint)
    children = []
    a_service_started = False
    stopping = False

    def stop_signal(*unused):
        nonlocal stopping
        stopping = True
        (run/'stop.request').touch()

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    state = {'manager': process_info(os.getpid()), 'domain': args.domain, 'port': args.port,
             'scene': str(profile['scene'].resolve()), 'scene_id': profile['id'],
             'scene_name': profile['name'], 'config': str(run/'scene_config.yaml'),
             'waypoints': str(run/'initial_waypts.yaml'),
             'flyco_source': str(ROOT/'third_party/flyco_ros2'),
             'flyco_runtime': str(flyco_runtime), 'docker': args.docker,
             'segmentation_backend': args.segmentation_backend,
             'segmentation_endpoint': args.segmentation_endpoint}
    (run/'session.json').write_text(json.dumps(state, indent=2))
    output = ConsoleOutput(run, verbose=args.verbose)
    try:
        if not args.no_perception and args.segmentation_backend == 'semantic':
            import semantic_perception_service as sam_a_service
            if args.segmentation_endpoint == sam_a_service.ENDPOINT:
                print('正在准备语义目标感知服务。', flush=True)
                service = sam_a_service.start()
                a_service_started = service['started']
                (run/'segmentation_service.json').write_text(json.dumps(service['health'], indent=2))
            else:
                info = sam_a_service.health(args.segmentation_endpoint)
                if info.get('backend') != 'A' or info.get('status') != 'ready':
                    raise RuntimeError('Configured semantic perception service is not ready')
        print(f'场景 {profile["id"]}: {profile["name"]}；正在启动 Isaac；日志目录：'+str(run), flush=True)
        sim_log = (run/'isaac.log').open('wb')
        cmd = [str(ROOT/'isaac_python.sh'), '-u', str(ROOT/'sim/flyco_app.py'),
            '--scene', str(profile['scene'].resolve()), '--config', str(run/'scene_config.yaml'),
            '--runtime', str(run), '--port', str(args.port),
            '--duration', str(args.duration), '--/app/extensions/fsWatcherEnabled=false',
            '--/log/file='+str(run/'kit.log')]
        if args.headless:
            cmd.append('--headless')
        if args.docker:
            cmd = container_runtime.command('isaac', cmd, run, env)
        sim = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=sim_log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        children.append(sim)
        deadline = time.monotonic()+(300. if args.docker else 120.)
        while not (run/'sim_metadata.json').exists():
            output.emit()
            if stopping or (run/'stop.request').exists():
                return 0
            returncode = sim.poll()
            timed_out = time.monotonic() > deadline
            if returncode is not None or timed_out:
                diagnostic = startup_failure(run, returncode, timed_out)
                state['startup_failure'] = diagnostic['reason']
                raise RuntimeError(diagnostic['message']+' See '+diagnostic['log_path'])
            time.sleep(.2)
        print('煤仓场景已加载，正在启动 FlyCo ROS2 算法。', flush=True)
        ros_log = (run/'ros.log').open('wb')
        ros_command = ['bash', str(ROOT/'tools/run_ros.sh')]
        if args.docker:
            ros_command = container_runtime.command('flyco', ros_command, run, env)
        ros = subprocess.Popen(ros_command, cwd=ROOT, env=env,
            stdout=ros_log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(ros)
        state.update(simulator=process_info(sim.pid), ros=process_info(ros.pid))
        (run/'session.json').write_text(json.dumps(state, indent=2))
        print('FlyCo Isaac session:', run, flush=True)
        while sim.poll() is None and not stopping:
            output.emit()
            if ros.poll() is not None:
                if (run/'result.json').exists() or (run/'stop.request').exists():
                    sim.wait(timeout=15)
                    break
                raise RuntimeError('FlyCo ROS stack exited; see '+str(run/'ros.log'))
            time.sleep(.3)
        if sim.poll() is None:
            try:
                sim.wait(timeout=15)
            except subprocess.TimeoutExpired:
                pass
        return sim.returncode or 0
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in reversed(children):
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=10)
        if args.docker:
            for role in ('flyco', 'isaac'):
                container_runtime.stop(role, run)
        if a_service_started:
            sam_a_service.stop()
        state['ended'] = time.time()
        (run/'session.json').write_text(json.dumps(state, indent=2))
        output.emit()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
