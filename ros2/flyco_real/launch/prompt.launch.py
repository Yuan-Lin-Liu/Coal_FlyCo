"""Open the text prompt window separately from the running flight algorithms."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def build(context):
    simulation = LaunchConfiguration('simulation').perform(context).lower() == 'true'
    return [Node(package='flyco_ros2', executable='prompt_ui_cv_node.py', name='prompt_ui',
        parameters=[{'use_sim_time': simulation, 'initial_text_prompt': ParameterValue(
            LaunchConfiguration('text'), value_type=str)}],
        remappings=[('/img_topic', '/isaac/camera/rgb' if simulation else '/flyco/camera/rgb')],
        output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('text', default_value='A single large pile of black coal.'),
        OpaqueFunction(function=build),
    ])
