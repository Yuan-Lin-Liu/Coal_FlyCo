"""Odin driver, calibration and localization in an independent ROS2 terminal."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('flyco_real'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config/odin.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/hardware.launch.py')),
            launch_arguments={'role': 'sensors', 'config': LaunchConfiguration('config')}.items())])
