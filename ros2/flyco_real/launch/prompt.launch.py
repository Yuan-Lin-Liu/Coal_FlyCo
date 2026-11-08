"""Ground-only camera/text UI, independent of flight and model processes."""
import json
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def build(context):
    simulation = LaunchConfiguration('simulation').perform(context).lower() == 'true'
    domain = LaunchConfiguration('domain').perform(context)
    node_env = {}
    if simulation and not domain:
        root = Path(LaunchConfiguration('project_root').perform(context)).expanduser()
        session_path = root/'runtime/latest/session.json'
        session = json.loads(session_path.read_text()) if session_path.is_file() else {}
        domain = session.get('domain', os.environ.get('ROS_DOMAIN_ID', '64'))
    if simulation or domain:
        node_env = {'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '0'}
    return [Node(package='flyco_ros2', executable='prompt_ui_cv_node.py', name='prompt_ui',
        parameters=[{'use_sim_time': simulation, 'initial_text_prompt': ParameterValue(
            LaunchConfiguration('text'), value_type=str)}],
        remappings=[('/img_topic', '/isaac/camera/rgb' if simulation else '/flyco/camera/rgb')],
        additional_env=node_env, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('project_root', default_value=os.environ.get('COAL_FLYCO_ROOT', str(Path.cwd()))),
        DeclareLaunchArgument('domain', default_value='',
                             description='Match the flight computer ROS domain; empty uses the environment or local simulation session'),
        DeclareLaunchArgument('text', default_value='A single large pile of black coal.'),
        OpaqueFunction(function=build),
    ])
