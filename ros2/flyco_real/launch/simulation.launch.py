"""ROS2 entrypoint for the existing Isaac/PX4/QGC simulation manager."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    def enabled(name):
        return arg(name).lower() == 'true'
    root = Path(arg('project_root')).expanduser().resolve()
    if not (root/'launch.py').is_file():
        raise FileNotFoundError('Set project_root to the coal_flyco repository: '+str(root))
    command = ['/usr/bin/python3', str(root/'launch.py'), 'run',
               '--flight-controller', 'px4-native', '--scene', arg('scene'),
               '--domain', arg('domain'), '--px4-dir', arg('px4_dir'),
               '--qgc-path', arg('qgc_path'), '--rviz' if enabled('rviz') else '--no-rviz']
    if arg('px4_msgs_setup'):
        command += ['--px4-msgs-setup', arg('px4_msgs_setup')]
    if not enabled('start_mode'):
        command += ['--external-native-mode']
    if not enabled('start_algorithms'):
        command += ['--external-algorithms']
    for option, flag in (('qgc', '--qgc'), ('headless', '--headless')):
        if enabled(option):
            command.append(flag)
    process = ExecuteProcess(cmd=command, cwd=str(root), output='screen',
                             sigterm_timeout='105', sigkill_timeout='10')
    return [process, RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Simulation manager exited'))]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=os.environ.get('COAL_FLYCO_ROOT', str(Path.cwd()))),
        DeclareLaunchArgument('scene', default_value='4'),
        DeclareLaunchArgument('domain', default_value='64'),
        DeclareLaunchArgument('px4_dir', default_value=str(Path.home()/'PX4-Autopilot')),
        DeclareLaunchArgument('px4_msgs_setup', default_value=''),
        DeclareLaunchArgument('qgc_path', default_value=str(Path.home()/'QGroundControl-x86_64.AppImage')),
        DeclareLaunchArgument('qgc', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_mode', default_value='false'),
        DeclareLaunchArgument('start_algorithms', default_value='false'),
        OpaqueFunction(function=build),
    ])
