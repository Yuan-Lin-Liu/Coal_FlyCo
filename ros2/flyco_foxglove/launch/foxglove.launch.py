"""Separate visualization entrypoint; leaves the existing RViz and flight launches intact."""
import os
from pathlib import Path
import shutil
from urllib.parse import urlencode

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from flyco_foxglove.converter import convert_rviz, topic_whitelist


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    source = Path(arg('rviz_config')).expanduser().resolve()
    output = Path(arg('output_path')).expanduser().resolve()
    _, report = convert_rviz(yaml.safe_load(source.read_text()))
    port = int(arg('port'))
    if not 1 <= port <= 65535:
        raise ValueError('port must be between 1 and 65535')
    environment = {'ROS_DOMAIN_ID': arg('domain')}
    parameters = {
        'address': arg('address'), 'port': port,
        'topic_whitelist': topic_whitelist(report['topics']),
        'capabilities': ['connectionGraph'],
        'client_topic_whitelist': ['^$'], 'service_whitelist': ['^$'],
        'param_whitelist': ['^$'], 'sysinfo': False,
    }
    actions = [
        Node(package='flyco_foxglove', executable='rviz_to_foxglove',
             name='rviz_to_foxglove', output='screen', additional_env=environment,
             parameters=[{'rviz_config': str(source), 'output_path': str(output)}]),
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='flyco_foxglove_bridge', output='screen', additional_env=environment,
             parameters=[parameters]),
        LogInfo(msg=f'Foxglove connection: ws://127.0.0.1:{port}; layout: {output}'),
    ]
    if arg('desktop').lower() == 'true':
        desktop = shutil.which('foxglove-studio')
        if desktop is None:
            raise FileNotFoundError('Install Foxglove Desktop from https://foxglove.dev/download, or use desktop:=false')
        url = 'foxglove://open?' + urlencode({
            'ds': 'foxglove-websocket', 'ds.url': f'ws://127.0.0.1:{port}',
        })
        actions.append(ExecuteProcess(cmd=[desktop, url], output='screen'))
    return actions


def generate_launch_description():
    root = Path(os.environ.get('COAL_FLYCO_ROOT', Path.cwd()))
    return LaunchDescription([
        DeclareLaunchArgument('domain', default_value=os.environ.get('ROS_DOMAIN_ID', '66')),
        DeclareLaunchArgument('address', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('desktop', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(
            Path(get_package_share_directory('flyco_real')) / 'config/flyco_real.rviz')),
        DeclareLaunchArgument('output_path', default_value=str(root/'runtime/foxglove/flyco_odin.json')),
        OpaqueFunction(function=build),
    ])
