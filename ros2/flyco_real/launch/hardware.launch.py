"""Single entrypoint for the bundled Odin sensor stack and native FlyCo roles."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from flyco_real.bringup import prepare_parameters


def build(context):
    config = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    role = LaunchConfiguration('role').perform(context)
    if role not in ('onboard', 'ground', 'sensors'):
        raise ValueError('role must be onboard, ground or sensors')
    settings = yaml.safe_load(config.read_text())
    deployment = settings.get('deployment', {})
    share = Path(get_package_share_directory('flyco_real'))
    common = {'config': str(config), 'runtime_root': LaunchConfiguration('runtime_root').perform(context)}
    if role == 'ground':
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/native.launch.py')),
                launch_arguments={**common, 'role': 'ground', 'ui': LaunchConfiguration('ui').perform(context)}.items())]
    _, run, params = prepare_parameters(get_package_share_directory('flyco_ros2'),
        Path(get_package_prefix('flyco_ros2')) / 'lib/flyco_ros2', config, common['runtime_root'])
    odin_share = Path(get_package_share_directory('odin_ros_driver'))
    control = yaml.safe_load((odin_share / 'config/control_command.yaml').read_text())
    control['register_keys'].update(settings.get('odin', {}).get('register_keys', {}))
    control_file = run / 'odin_control.yaml'
    control_file.write_text(yaml.safe_dump(control, sort_keys=False))
    calibration = run / 'odin'
    calibration.mkdir()
    actions = [SetEnvironmentVariable('ODIN_CALIB_DIR', str(calibration)),
               SetEnvironmentVariable('ODIN_COMMAND_FILE', str(calibration / 'command.txt'))]
    critical = []
    if LaunchConfiguration('start_driver').perform(context).lower() == 'true':
        driver = Node(package='odin_ros_driver', executable='host_sdk_sample', name='odin_driver',
                      parameters=[{'config_file': str(control_file)}], output='screen')
        actions.append(driver)
        critical.append(driver)
    adapter = Node(package='flyco_real', executable='odin_adapter.py', name='odin_adapter',
                   parameters=[str(params)], output='screen')
    actions.append(adapter)
    critical.append(adapter)
    if role == 'sensors':
        sensor = Node(package='flyco_real', executable='sensor_bridge.py', name='sensor_bridge',
                      parameters=[str(params)], output='screen')
        actions.append(sensor)
        critical.append(sensor)
    else:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/native.launch.py')),
            launch_arguments={**common, 'role': 'onboard', 'runtime_root': str(run / 'algorithm')}.items()))
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Odin sensor provider exited'))])))
    print('Odin native %s; runtime: %s' % (role, run), flush=True)
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('flyco_real'))
    return LaunchDescription([
        DeclareLaunchArgument('role', default_value='onboard'),
        DeclareLaunchArgument('config', default_value=str(share / 'config/odin.yaml')),
        DeclareLaunchArgument('runtime_root', default_value=str(Path.home() / '.ros/coal-flyco')),
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('ui', default_value='false'),
        OpaqueFunction(function=build),
    ])
