"""Start the persistent FlyCo registration/control node in its own terminal."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
                 'auto_enable': LaunchConfiguration('auto_enable').perform(context).lower() == 'true'}
    if simulation:
        overrides.update(odom_topic='/isaac/odom', bounds_min=[-32., -24., -.5],
                         bounds_max=[32., 24., 10.], px4_local_timeout=1.)
    return [Node(package='flyco_real', executable='px4_bridge.py', name='px4_bridge',
                 parameters=[params, overrides], output='screen')]


def generate_launch_description():
    share = Path(get_package_share_directory('flyco_real'))
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('auto_enable', default_value='true'),
        DeclareLaunchArgument('config', default_value=str(share/'config/odin.yaml')),
        OpaqueFunction(function=build),
    ])
