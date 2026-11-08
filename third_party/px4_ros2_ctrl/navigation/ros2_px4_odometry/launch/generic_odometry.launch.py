#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the shared directory of the package
    pkg_share = get_package_share_directory('ros2_px4_odometry')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='odin_config.yaml',
        description='Determine which external odometry source to publish to PX4'
    )

    # Launch the generic odometry node
    generic_odometry_node = Node(
        package='ros2_px4_odometry',
        executable='odom_to_px4_node',
        parameters=[
            # Load the configuration file
            PathJoinSubstitution([
                pkg_share, 'config', LaunchConfiguration('config_file')
            ])
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,

        generic_odometry_node,
    ])
