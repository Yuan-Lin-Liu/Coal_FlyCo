"""Run the semantic inference HTTP service.

The service can run on the ground computer while the tracking node runs on
Orin.  In that deployment the bind address must be reachable from Orin.
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = os.environ.get('COAL_FLYCO_ROOT', str(Path.cwd()))
    return LaunchDescription([
        DeclareLaunchArgument('source_dir', default_value=root+'/third_party/SAM_AD_Perception'),
        DeclareLaunchArgument('python', default_value=root+'/third_party/SAM_AD_Perception/.venv/bin/python'),
        DeclareLaunchArgument('bind', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8851'),
        ExecuteProcess(cmd=[LaunchConfiguration('python'), '-B', '-m', 'sam_ad.server',
            '--backend', 'A', '--bind', LaunchConfiguration('bind'),
            '--port', LaunchConfiguration('port')],
            cwd=LaunchConfiguration('source_dir'), output='screen')])
