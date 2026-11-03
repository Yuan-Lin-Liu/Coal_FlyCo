"""Docker subprocesses sharing the existing native launcher's session protocol."""
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CONTAINER_ROOT = '/workspace/coal_flyco'


def image(role):
    return os.environ.get('COAL_' + role.upper() + '_IMAGE', 'coal-flyco/' + role + ':5.1.0-v1')


def container_name(role, run):
    return 'coal-flyco-' + run.name + '-' + role


def map_path(value):
    return str(value).replace(str(ROOT), CONTAINER_ROOT)


def command(role, argv, run, env, suffix=""):
    cache = ROOT / '.local/docker' / role
    cache.mkdir(parents=True, exist_ok=True)
    models = ROOT / '.local/flyco/models'
    models.mkdir(parents=True, exist_ok=True)
    result = ['docker', 'run', '--rm', '--init', '--network', 'host', '--ipc', 'host',
              '--gpus', 'all', '--stop-timeout', '20',
              '--name', container_name(role, run) + suffix,
              '--user', str(os.getuid()) + ':' + str(os.getgid()),
              '--workdir', CONTAINER_ROOT,
              '--volume', str(ROOT) + ':' + CONTAINER_ROOT + ':ro',
              '--volume', str(ROOT / 'runtime') + ':' + CONTAINER_ROOT + '/runtime:rw',
              '--volume', str(cache) + ':/home/coal:rw',
              '--env', 'HOME=/home/coal', '--env', 'NVIDIA_DRIVER_CAPABILITIES=all']
    if role == 'flyco':
        result += ['--volume', str(models) + ':/models:ro',
                   '--env', 'FLYCO_RUNTIME=/opt/flyco-runtime',
                   '--env', 'FLYCO_MODELS=/models', '--env', 'FLYCO_CACHE=/home/coal/.cache/flyco']
    else:
        if os.environ.get('ACCEPT_EULA') != 'Y':
            raise RuntimeError('Set ACCEPT_EULA=Y after accepting the NVIDIA Isaac Sim EULA.')
        result += ['--env', 'ACCEPT_EULA=Y', '--env', 'ISAACSIM_PATH=/isaac-sim']
        # Kit's portable Python application writes beside its executable,
        # independently of HOME. Persist those paths for the invoking UID.
        for folder in ('cache', 'data', 'logs'):
            destination = cache / ('kit-' + folder)
            destination.mkdir(parents=True, exist_ok=True)
            result += ['--volume', str(destination) + ':/isaac-sim/kit/' + folder + ':rw']
    for key, value in env.items():
        if key.startswith('FLYCO_ISAAC_'):
            result += ['--env', key + '=' + map_path(value)]
    if env.get('DISPLAY'):
        result += ['--env', 'DISPLAY=' + env['DISPLAY'],
                   '--volume', '/tmp/.X11-unix:/tmp/.X11-unix:ro']
        auth = Path(env.get('XAUTHORITY') or str(Path.home() / '.Xauthority'))
        if auth.is_file():
            result += ['--volume', str(auth) + ':/tmp/coal.Xauthority:ro',
                       '--env', 'XAUTHORITY=/tmp/coal.Xauthority']
    if Path('/dev/dri').exists():
        result += ['--device', '/dev/dri']
        for gid in sorted({device.stat().st_gid for device in Path('/dev/dri').iterdir()}):
            result += ['--group-add', str(gid)]
    return result + [image(role)] + [map_path(arg) for arg in argv]


def preflight():
    if os.environ.get('ACCEPT_EULA') != 'Y':
        raise RuntimeError('Set ACCEPT_EULA=Y after accepting the NVIDIA Isaac Sim EULA.')
    subprocess.run(['docker', 'info', '--format', '{{.ServerVersion}}'], check=True,
                   stdout=subprocess.DEVNULL)
    for role in ('isaac', 'flyco'):
        if subprocess.run(['docker', 'image', 'inspect', image(role)],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
            raise RuntimeError('Missing image ' + image(role) + '; run ./docker/setup.sh first.')


def stop(role, run):
    subprocess.run(['docker', 'stop', '--time', '10', container_name(role, run)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)
