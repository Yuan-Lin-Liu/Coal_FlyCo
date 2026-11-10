"""Original Odin driver TF and zero-offset localization in a ROS2 terminal."""
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
        DeclareLaunchArgument('odometry_config', default_value=str(
            Path(get_package_share_directory('ros2_px4_odometry'))/'config/odin_config.yaml')),
        DeclareLaunchArgument('start_odometry', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/hardware.launch.py')),
            launch_arguments={'role': 'sensors', 'config': LaunchConfiguration('config'),
                              'odometry_config': LaunchConfiguration('odometry_config'),
                              'start_odometry': LaunchConfiguration('start_odometry')}.items())])
