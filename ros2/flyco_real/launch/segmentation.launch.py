"""Ground image/text segmentation using the existing uv GPU worker, without HTTP."""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolved_settings(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    def path(value):
        # Resolving bin/python's symlink would bypass the uv environment.
        return os.path.abspath(os.path.expanduser(os.path.expandvars(value)))
    root = path(arg('project_root'))
    config = yaml.safe_load(Path(arg('config')).expanduser().read_text())
    settings = dict(config['ground_segmentation']['ros__parameters'])
    settings['source_dir'] = path(arg('source_dir') or str(Path(root) / 'third_party/SAM_AD_Perception'))
    settings['model_python'] = path(arg('model_python') or str(Path(root) / '.local/deploy/semantic/bin/python'))
    for name in ('image_topic', 'text_topic'):
        if arg(name):
            settings[name] = arg(name)
    if arg('text'):
        settings['initial_text'] = arg('text')
    return settings


def build(context):
    settings = resolved_settings(context)
    if not os.access(settings['model_python'], os.X_OK):
        raise FileNotFoundError('Semantic uv Python missing: ' + settings['model_python'])
    if not (Path(settings['source_dir']) / 'runtime_api/models.py').is_file():
        raise FileNotFoundError('Semantic sources missing: ' + settings['source_dir'])
    share = Path(get_package_share_directory('flyco_real'))
    segmentation = Node(package='flyco_real', executable='semantic_image.py',
                        name='ground_segmentation', parameters=[settings], output='screen')
    actions = [segmentation, RegisterEventHandler(OnProcessExit(
        target_action=segmentation,
        on_exit=[EmitEvent(event=Shutdown(reason='Ground segmentation node exited'))]))]
    if LaunchConfiguration('ui').perform(context).lower() == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/prompt.launch.py')),
            launch_arguments={'image_topic': settings['image_topic'],
                              'text_topic': settings['text_topic'],
                              'text': settings['initial_text']}.items()))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
            arguments=['-d', str(share / 'config/segmentation.rviz')],
            remappings=[('/odin1/image', settings['image_topic']),
                        ('/perception/segmentation/mask', settings['mask_topic']),
                        ('/perception/segmentation/overlay', settings['overlay_topic'])],
            parameters=[{'use_sim_time': False}], output='screen'))
    return actions


def generate_launch_description():
    root = os.environ.get('COAL_FLYCO_ROOT', str(Path.cwd()))
    share = Path(get_package_share_directory('flyco_real'))
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=root),
        DeclareLaunchArgument('config', default_value=str(share / 'config/segmentation.yaml')),
        DeclareLaunchArgument('source_dir', default_value=''),
        DeclareLaunchArgument('model_python', default_value=''),
        DeclareLaunchArgument('image_topic', default_value=''),
        DeclareLaunchArgument('text_topic', default_value=''),
        DeclareLaunchArgument('text', default_value=''),
        DeclareLaunchArgument('ui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=build),
    ])
