"""Bring up the flight stack; text arrives from the independent ground UI."""
import json
import os
import time
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context, state=None):
    if state is None:
        state = {'simulation_started': False, 'deadline': time.monotonic()+180.}
    def arg(name): return LaunchConfiguration(name).perform(context)
    simulation = arg('simulation').lower() == 'true'
    share = Path(get_package_share_directory('flyco_real'))
    actions, critical = [], []
    node_env = {}
    mission_params = {'use_sim_time': simulation, 'takeoff_on_armed_flyco': True, 'auto_scan': True}
    gate_params = {'use_sim_time': simulation}
    if simulation:
        root = Path(arg('project_root')).expanduser().resolve()
        run = Path(arg('session_dir') or str(root/'runtime/latest')).expanduser().resolve()
        session_path = run/'session.json'
        session = json.loads(session_path.read_text()) if session_path.is_file() else {}
        if not session or session.get('ended'):
            if not state['simulation_started']:
                state.update(simulation_started=True, deadline=time.monotonic()+180.)
                simulator = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/simulation.launch.py')),
                    launch_arguments={**{name: arg(name) for name in ('project_root', 'scene', 'domain', 'px4_dir',
                        'headless', 'rviz', 'qgc', 'qgc_path')}, 'start_mode': 'false', 'start_algorithms': 'false'}.items())
                return [GroupAction(actions=[simulator]), TimerAction(period=1., actions=[OpaqueFunction(function=build, kwargs={'state': state})])]
            if time.monotonic() > state['deadline']:
                raise RuntimeError('Simulation startup timed out; inspect the simulation log')
            return [TimerAction(period=1., actions=[OpaqueFunction(function=build, kwargs={'state': state})])]
        if session.get('flight_controller') != 'px4-native':
            raise RuntimeError('Stop the existing non-native simulation before starting FlyCo')
        if not session.get('external_algorithms'):
            raise RuntimeError('Terminal A already runs algorithms; restart it with start_algorithms:=false')
        if not (run/'sim_metadata.json').is_file() or not (run/'sim_status.json').is_file():
            if time.monotonic() > state.get('deadline', time.monotonic()+180.):
                raise RuntimeError('Simulation sensor startup timed out')
            return [TimerAction(period=1., actions=[OpaqueFunction(function=build, kwargs={'state': state})])]
        env = dict(FLYCO_ISAAC_RUNTIME=str(run), FLYCO_ISAAC_PORT=str(session['port']),
            FLYCO_ISAAC_DOMAIN=str(session['domain']), FLYCO_ISAAC_STACK_ROLE='algorithms',
            FLYCO_ISAAC_FLIGHT_CONTROLLER='px4-native', FLYCO_ISAAC_EXTERNAL_NATIVE_MODE='1',
            FLYCO_PX4_MSGS_SETUP=session.get('px4_msgs_setup', ''), FLYCO_ISAAC_RVIZ='0',
            FLYCO_ISAAC_NO_PERCEPTION='0',
            FLYCO_ISAAC_NO_UI='1',
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
                              'start_sensors': 'false',
                              'segmentation_endpoint': arg('segmentation_endpoint'),
                              'segmentation_timeout': arg('segmentation_timeout')}.items())
        if arg('start_semantic').lower() == 'true':
            root = Path(arg('project_root')).expanduser().resolve()
            try:
                settings = yaml.safe_load(Path(arg('config')).expanduser().read_text()) or {}
            except (OSError, yaml.YAMLError):
                settings = {}
            deployment = settings.get('deployment', {})
            semantic_root = Path(arg('semantic_root') or str(root / 'third_party/SAM_AD_Perception')).expanduser()
            semantic_python = Path(arg('semantic_python') or str(semantic_root / '.venv/bin/python')).expanduser()
            semantic = ExecuteProcess(
                cmd=[str(semantic_python), '-B', '-m', 'sam_ad.server', '--backend', 'A',
                     '--bind', str(deployment.get('semantic_bind', '127.0.0.1')),
                     '--port', str(deployment.get('semantic_port', 8851))],
                cwd=str(semantic_root), name='semantic_perception', output='screen')
            actions.append(semantic)
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=semantic,
                on_exit=[EmitEvent(event=Shutdown(reason='Semantic perception service exited'))])))
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
                arguments=['wait-armed'],
                parameters=[gate_params], output='screen')
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
    px4_dir = next((p for p in (Path.home()/'PX4-Autopilot', Path.home()/'PX4')
                    if (p/'build/px4_sitl_default/bin/px4').is_file()), Path.home()/'PX4-Autopilot')
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('config', default_value=str(share/'config/odin.yaml')),
        DeclareLaunchArgument('project_root', default_value=os.environ.get('COAL_FLYCO_ROOT', str(Path.cwd()))),
        DeclareLaunchArgument('session_dir', default_value=''),
        DeclareLaunchArgument('scene', default_value='4'),
        DeclareLaunchArgument('domain', default_value='64'),
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('PX4_DIR', str(px4_dir))),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_mode', default_value='true',
                             description='Register FlyCo here; false only when a separate mode node is already running'),
        DeclareLaunchArgument('start_semantic', default_value='true'),
        DeclareLaunchArgument('semantic_root', default_value=''),
        DeclareLaunchArgument('semantic_python', default_value=''),
        DeclareLaunchArgument('segmentation_endpoint', default_value='',
                              description='Override the HTTP semantic service endpoint for onboard tracking'),
        DeclareLaunchArgument('segmentation_timeout', default_value='',
                              description='Override the HTTP semantic request timeout in seconds'),
        OpaqueFunction(function=build, kwargs={'state': {'simulation_started': False, 'deadline': time.monotonic()+180.}}),
    ])
