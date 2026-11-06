"""Independent Orin / ground entrypoint. The hardware launch provides Odin; PX4 uses the existing DDS agent."""
import os
import shlex
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from flyco_real.bringup import prepare_parameters


def build(context):
    role = LaunchConfiguration('role').perform(context)
    if role not in ('onboard', 'ground'):
        raise ValueError('role must be onboard or ground')
    share = Path(get_package_share_directory('flyco_ros2'))
    binaries = Path(get_package_prefix('flyco_ros2')) / 'lib/flyco_ros2'
    hardware = LaunchConfiguration('config').perform(context)
    params, run, config = prepare_parameters(share, binaries, hardware,
        LaunchConfiguration('runtime_root').perform(context))
    for executable in (('flyco_planner', 'traj_server', 'sam_tracking_node', 'LKH', 'kdline')
                       if role == 'onboard' else ()):
        if not os.access(binaries / executable, os.X_OK):
            raise FileNotFoundError('Build the native workspace first: ' + str(binaries / executable))
    specs = yaml.safe_load((Path(get_package_share_directory('flyco_real')) / 'config/nodes.yaml').read_text())
    specs = {s['name']: s for s in specs}
    actions, critical = [], []
    python = LaunchConfiguration('python_executable').perform(context)
    def node(package, executable, name, remappings=(), extra=None):
        action = Node(package=package, executable=executable, name=name,
                      parameters=[str(config), extra or {}], remappings=list(remappings), output='screen',
                      prefix=[shlex.quote(python)] if python and executable.endswith('.py') else None)
        actions.append(action)
        critical.append(action)
        return action
    def flyco(name, extra=None):
        spec = specs[name]
        remaps = list(spec.get('remappings', {}).items())
        return node('flyco_ros2', spec['executable'], name, remaps, extra)
    if role == 'onboard':
        if LaunchConfiguration('start_sensor_bridge').perform(context).lower() == 'true':
            node('flyco_real', 'sensor_bridge.py', 'sensor_bridge')
            fov = node('flyco_real', 'camera_fov.py', 'camera_fov')
            critical.remove(fov)
        if LaunchConfiguration('start_mode').perform(context).lower() == 'true':
            node('flyco_real', 'px4_bridge.py', 'px4_bridge')
        flyco('flyco_planner')
        flyco('traj_server')
        flyco('sam_tracking_node')
        node('flyco_real', 'tracking_tf.py', 'sam_tracking')
        flyco('tracking_prompt')
    else:
        flyco('mesh_predict_node')
        node('flyco_real', 'task.py', 'ground_heartbeat', extra={'heartbeat': True})
        if LaunchConfiguration('ui').perform(context).lower() == 'true':
            ui = flyco('prompt_ui')
            critical.remove(ui)
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A native FlyCo process exited'))])))
    print('Native FlyCo %s; runtime/config: %s' % (role, run), flush=True)
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('flyco_real'))
    return LaunchDescription([
        DeclareLaunchArgument('role', default_value='onboard'),
        DeclareLaunchArgument('config', default_value=str(share / 'config/hardware.yaml')),
        DeclareLaunchArgument('runtime_root', default_value=str(Path.home() / '.ros/coal-flyco')),
        DeclareLaunchArgument('ui', default_value='false'),
        DeclareLaunchArgument('start_sensor_bridge', default_value='true'),
        DeclareLaunchArgument('start_mode', default_value='true'),
        DeclareLaunchArgument('python_executable', default_value=''),
        OpaqueFunction(function=build),
    ])
