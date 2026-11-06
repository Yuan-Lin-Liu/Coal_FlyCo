"""Run FlyCo algorithms and automatic takeoff after Position -> FlyCo activation."""
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    def arg(name): return LaunchConfiguration(name).perform(context)
    simulation = arg('simulation').lower() == 'true'
    actions, critical = [], []
    node_env = {}
    mission_params = {'use_sim_time': simulation}
    if simulation:
        root = Path(arg('project_root')).expanduser().resolve()
        run = Path(arg('session_dir') or str(root/'runtime/latest')).expanduser().resolve()
        session = json.loads((run/'session.json').read_text())
        if session.get('ended') or session.get('flight_controller') != 'px4-native':
            raise RuntimeError('Start an active native simulation in terminal A first')
        if not session.get('external_algorithms'):
            raise RuntimeError('Terminal A already runs algorithms; restart it with start_algorithms:=false')
        if not (run/'sim_metadata.json').is_file():
            raise RuntimeError('Wait for the Isaac scene and sensor bridge before starting FlyCo')
        env = dict(FLYCO_ISAAC_RUNTIME=str(run), FLYCO_ISAAC_PORT=str(session['port']),
            FLYCO_ISAAC_DOMAIN=str(session['domain']), FLYCO_ISAAC_STACK_ROLE='algorithms',
            FLYCO_ISAAC_FLIGHT_CONTROLLER='px4-native', FLYCO_ISAAC_EXTERNAL_NATIVE_MODE='1',
            FLYCO_PX4_MSGS_SETUP=session.get('px4_msgs_setup', ''), FLYCO_ISAAC_RVIZ='0',
            FLYCO_ISAAC_NO_PERCEPTION='0',
            FLYCO_ISAAC_NO_UI='0' if arg('ui').lower() == 'true' else '1',
            FLYCO_ISAAC_SEGMENTATION_BACKEND=session['segmentation_backend'],
            FLYCO_ISAAC_SEGMENTATION_ENDPOINT=session['segmentation_endpoint'])
        algorithms = ExecuteProcess(cmd=['bash', str(root/'tools/run_ros.sh')],
                                    cwd=str(root), additional_env=env, output='screen')
        actions.append(algorithms)
        critical.append(algorithms)
        mission_params.update(odom_topic='/isaac/odom', gimbal_topic='/gimbal_pose')
        node_env = {'ROS_DOMAIN_ID': str(session['domain']), 'ROS_LOCALHOST_ONLY': '0'}
    else:
        share = Path(get_package_share_directory('flyco_real'))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/native.launch.py')),
            launch_arguments={'role': 'onboard', 'config': arg('config'), 'start_mode': 'false',
                              'start_sensor_bridge': 'false'}.items()))
    mission = Node(package='flyco_real', executable='mission.py', name='flyco_mission',
                   parameters=[mission_params], additional_env=node_env, output='screen')
    actions.append(mission)
    critical.append(mission)
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='FlyCo algorithm or mission node exited'))])))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('flyco_real'))
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('config', default_value=str(share/'config/odin.yaml')),
        DeclareLaunchArgument('project_root', default_value=os.environ.get('COAL_FLYCO_ROOT', str(Path.cwd()))),
        DeclareLaunchArgument('session_dir', default_value=''),
        DeclareLaunchArgument('ui', default_value='false'),
        OpaqueFunction(function=build),
    ])
