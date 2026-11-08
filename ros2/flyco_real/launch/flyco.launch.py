"""Register FlyCo, then start algorithms and the mission after normal arming."""
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    def arg(name): return LaunchConfiguration(name).perform(context)
    simulation = arg('simulation').lower() == 'true'
    share = Path(get_package_share_directory('flyco_real'))
    actions, critical = [], []
    node_env = {}
    mission_params = {'use_sim_time': simulation, 'takeoff_on_armed_flyco': True}
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
        critical.append(algorithms)
        mission_params.update(odom_topic='/isaac/odom', gimbal_topic='/gimbal_pose')
        node_env = {'ROS_DOMAIN_ID': str(session['domain']), 'ROS_LOCALHOST_ONLY': '0'}
    else:
        algorithms = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/native.launch.py')),
            launch_arguments={'role': 'onboard', 'config': arg('config'), 'start_mode': 'false',
                              'start_sensors': 'false'}.items())
    mission = Node(package='flyco_real', executable='mission.py', name='flyco_mission',
                   parameters=[mission_params], additional_env=node_env, output='screen')
    critical.append(mission)
    startup = []
    for process in critical:
        startup.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='FlyCo algorithm or mission node exited'))])))
    startup.extend([algorithms, mission])
    if arg('start_mode').lower() == 'true':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/mode.launch.py')),
            launch_arguments={'simulation': str(simulation).lower(), 'config': arg('config')}.items()))
    gate = Node(package='flyco_real', executable='task.py', name='flyco_arming_gate',
                arguments=['wait-armed'], parameters=[{'use_sim_time': simulation}], output='screen')
    def after_arming(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='FlyCo arming wait failed'))]
        return [LogInfo(msg='FlyCo armed; launching algorithms and mission'), *startup]
    actions.extend([RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_arming)), gate])
    # Registration, arming wait and mission must share the simulator's DDS domain
    # even when the operator starts this command from an unconfigured terminal.
    environment = [SetEnvironmentVariable(k, v) for k, v in node_env.items()]
    return [GroupAction(actions=environment+actions)]


def generate_launch_description():
    share = Path(get_package_share_directory('flyco_real'))
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('config', default_value=str(share/'config/odin.yaml')),
        DeclareLaunchArgument('project_root', default_value=os.environ.get('COAL_FLYCO_ROOT', str(Path.cwd()))),
        DeclareLaunchArgument('session_dir', default_value=''),
        DeclareLaunchArgument('ui', default_value='false'),
        DeclareLaunchArgument('start_mode', default_value='true',
                             description='Register FlyCo here; false only when a separate mode node is already running'),
        OpaqueFunction(function=build),
    ])
