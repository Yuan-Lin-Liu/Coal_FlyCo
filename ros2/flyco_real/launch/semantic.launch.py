"""Run semantic inference on Orin; mesh prediction remains on the ground PC."""
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
        DeclareLaunchArgument('port', default_value='8851'),
        ExecuteProcess(cmd=[LaunchConfiguration('python'), '-B', '-m', 'sam_ad.server',
            '--backend', 'A', '--bind', '127.0.0.1', '--port', LaunchConfiguration('port')],
            cwd=LaunchConfiguration('source_dir'), output='screen')])
