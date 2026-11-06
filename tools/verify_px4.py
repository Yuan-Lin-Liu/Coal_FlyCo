#!/usr/bin/env python3
"""Verify the PX4 mission handoff in one Isaac session, then land and stop.

Uses real perception, planning and SITL. Scan verification checks sustained
motion and trajectory handoff; it does not claim complete surface coverage.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=('4', '6'), default='4')
    parser.add_argument('--qgc', action='store_true')
    parser.add_argument('--landing-only', action='store_true',
                        help='Use takeoff only as setup to verify the remaining landing path')
    parser.add_argument('--qgc-land', action='store_true',
                        help='Wait for a Land operation in QGC instead of sending the backend command')
    parser.add_argument('--prompt', default='A single large pile of black coal.')
    parser.add_argument('--scan-seconds', type=float, default=20.)
    args = parser.parse_args()
    if args.qgc_land and not args.qgc:
        parser.error('--qgc-land requires --qgc')
    if args.qgc_land and shutil.which('gdbus'):
        lock = subprocess.run(['gdbus', 'call', '--session', '--dest', 'org.gnome.ScreenSaver',
            '--object-path', '/org/gnome/ScreenSaver', '--method', 'org.gnome.ScreenSaver.GetActive'],
            capture_output=True, text=True, timeout=5.)
        if lock.returncode == 0 and lock.stdout.strip() == '(true,)':
            parser.error('Unlock the desktop before QGC interface verification')
    folder = ROOT/'runtime'/('px4-verification-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    folder.mkdir(parents=True)
    report = {'ok': False, 'scope': 'takeoff, perception, scan handoff, hold/resume, landing',
              'coverage_completion_tested': False, 'stages': []}
    run = None
    command = [sys.executable, str(ROOT/'launch.py'), 'run', '--scene', args.scene,
               '--headless', '--no-ui', '--no-rviz']
    if args.qgc:
        command.append('--qgc')
    if args.landing_only:
        command.append('--no-perception')
        report['scope'] = 'landing only; takeoff is setup'
    with (folder/'manager.log').open('wb') as log:
        manager = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)

    def interrupted(*unused):
        raise KeyboardInterrupt('Verification interrupted; landing and saving results')
    signal.signal(signal.SIGTERM, interrupted)

    def wait(label, predicate, timeout=90.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if manager.poll() is not None:
                raise RuntimeError(f'{label}: session exited ({manager.returncode}); see {folder}/manager.log')
            value = read(run/'sim_status.json') if run else {}
            if predicate(value):
                report['stages'].append({'stage': label, 'status': value})
                (folder/'report.json').write_text(json.dumps(report, indent=2)+'\n')
                print(label, 'PASS', 't='+str(round(value.get('t', 0.), 2)), flush=True)
                return value
            time.sleep(.25)
        raise TimeoutError(f'{label}: exceeded {timeout}s; last status={value}')

    def action(name, **fields):
        path = run/'actions'/f'{time.time_ns()}-{uuid.uuid4().hex}.json'
        path.write_text(json.dumps(dict(action=name, **fields)))

    try:
        deadline = time.monotonic()+240.
        while time.monotonic() < deadline:
            latest = ROOT/'runtime/latest'
            session = read(latest/'session.json')
            if session.get('manager', {}).get('pid') == manager.pid:
                run = latest.resolve()
                report['run_dir'] = str(run)
                break
            if manager.poll() is not None:
                raise RuntimeError(f'Launcher exited; see {folder}/manager.log')
            time.sleep(.25)
        if run is None:
            raise TimeoutError('Session directory not created')
        wait('ready', lambda s: s.get('t', 0.) > 5. and
             s.get('px4', {}).get('local_position_ned') is not None and
             read(run/'bridge_status.json').get('ready_for_initialize'), 240.)
        action('initialize')
        wait('initialize', lambda s: s.get('px4', {}).get('offboard_stream') and
             read(run/'bridge_status.json').get('ready_for_initial'))
        action('initial')
        initial = wait('takeoff_endpoint', lambda s: s.get('px4', {}).get('endpoint_wait') and
                       abs(s['position'][2]-2.) < .15 and math.dist(s['velocity'], [0.,0.,0.]) < .2)
        if not args.landing_only:
            wait('perception_ready', lambda s: read(run/'bridge_status.json').get('perception_subscribers', 0) > 0)
            action('text', text=args.prompt)
            wait('predicted_mesh', lambda s: s.get('telemetry', {}).get('mesh_vertices', 0) > 0, 300.)
            action('plan')
            wait('planning_enabled', lambda s: s.get('telemetry', {}).get('last_action', {}).get('action') == 'plan')
            action('scan')
            scanning = wait('scan_handoff', lambda s: s.get('px4', {}).get('tracking') and
                            s.get('command_count', 0) > initial['command_count']+10 and
                            math.dist(s['position'], initial['position']) > .5, 240.)
            def scan_running(s):
                if s.get('fault'):
                    raise RuntimeError('Scan fault: '+s['fault'])
                return s.get('t', 0.) >= scanning['t']+args.scan_seconds
            wait('sustained_scan', scan_running, 180.)
            action('hold')
            held = wait('hold', lambda s: s.get('px4', {}).get('offboard_stream') and
                        not s.get('px4', {}).get('accept_trajectories', True) and
                        math.dist(s.get('velocity', [1.,1.,1.]), [0.,0.,0.]) < .2)
            action('resume')
            wait('resume', lambda s: s.get('t', 0.) > held['t'] and
                 s.get('px4', {}).get('accept_trajectories') and s.get('px4', {}).get('pending') is None)
        if args.qgc_land:
            (run/'awaiting_qgc_land').touch()
            print('Waiting for QGC Land; marker:', run/'awaiting_qgc_land', flush=True)
            wait('qgc_land_mode', lambda s: s.get('px4', {}).get('flight_mode') == 'LAND', 600.)
        else:
            action('land')
            wait('land_mode', lambda s: s.get('px4', {}).get('flight_mode') == 'LAND', 30.)
        wait('landed_and_stopped', lambda s: s.get('px4', {}).get('landing_complete') and
             not s['px4']['armed'] and max(map(abs, s['motors'])) < 1., 90.)
        report['ok'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc)
        print(str(exc), file=sys.stderr, flush=True)
    finally:
        if run and manager.poll() is None:
            (run/'stop.request').touch()
        try:
            manager.wait(timeout=125.)
        except subprocess.TimeoutExpired:
            manager.terminate()
            manager.wait(timeout=30.)
        report['manager_returncode'] = manager.returncode
        if run:
            report['session_result'] = read(run/'result.json')
        report['ok'] = report['ok'] and manager.returncode == 0
        (folder/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print('Report:', folder/'report.json', flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
