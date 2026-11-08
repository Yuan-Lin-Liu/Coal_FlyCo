"""Start the persistent FlyCo registration/control node in its own terminal."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    simulation = LaunchConfiguration('simulation').perform(context).lower() == 'true'
    config = LaunchConfiguration('config').perform(context)
    settings = yaml.safe_load(Path(config).expanduser().read_text()) if config else {}
    # Vehicle profiles also contain deployment/driver sections, which are not
    # ROS parameter-file syntax. Pass only this node's parameter dictionary.
    params = dict(settings.get('px4_bridge', {}).get('ros__parameters', {}))
    prefix = settings.get('deployment', {}).get('px4_topic_prefix')
    if prefix:
        params['px4_topic_prefix'] = prefix
    overrides = {'simulation': simulation, 'use_sim_time': simulation,
                 'check_odom_freshness': LaunchConfiguration('check_odom_freshness').perform(context).lower() == 'true',
                 'auto_enable': LaunchConfiguration('auto_enable').perform(context).lower() == 'true'}
    if simulation:
        overrides.update(odom_topic='/isaac/odom', bounds_min=[-32., -24., -.5],
                         bounds_max=[32., 24., 10.], px4_local_timeout=1.)
    mode = Node(package='flyco_mode', executable='flyco_mode', name='flyco_mode',
                parameters=[{'px4_topic_prefix': params.get('px4_topic_prefix', '/fmu'),
                             'use_sim_time': simulation}], output='screen')
    planner = Node(package='flyco_real', executable='px4_bridge.py', name='px4_bridge',
                   parameters=[params, overrides], output='screen')
    return [mode, planner] + [RegisterEventHandler(OnProcessExit(target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason='FlyCo mode process exited'))]))
        for process in (mode, planner)]


def generate_launch_description():
    share = Path(get_package_share_directory('flyco_real'))
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('auto_enable', default_value='true'),
        DeclareLaunchArgument('check_odom_freshness', default_value='true', choices=['true', 'false'],
                             description='Set false only for ground arming tests; disables FlyCo takeoff and tracking'),
        DeclareLaunchArgument('config', default_value=str(share/'config/odin.yaml')),
        OpaqueFunction(function=build),
    ])
