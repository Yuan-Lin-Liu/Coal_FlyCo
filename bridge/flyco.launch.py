"""Original FlyCo_ROS2 algorithms with an Isaac sensor provider."""
import json
import os
from pathlib import Path
import re
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    runtime = Path(os.environ['FLYCO_ISAAC_RUNTIME'])
    source = root/'third_party/flyco_ros2'
    flyco_runtime = Path(os.environ.get('FLYCO_RUNTIME', root/'.local/flyco'))
    models = Path(os.environ.get('FLYCO_MODELS', flyco_runtime/'models'))
    build = flyco_runtime/'build'
    meta = json.loads((runtime/'sim_metadata.json').read_text())
    sensor_config = meta['camera']
    values = {'evf_root': os.environ['FLYCO_EVF_ROOT'],
              'initial_text_prompt': sensor_config.get('initial_text_prompt', ''),
              'sam_checkpoint_dir': str(models/'sam2')}

    def replace(value):
        if isinstance(value, str):
            value = re.sub(r'\$\(\s*find-pkg-share\s+flyco_ros2\s*\)', str(source), value)
            value = re.sub(r'\$\(\s*var\s+([\w_]+)\s*\)',
                           lambda m: values[m[1]], value)
            return value
        if isinstance(value, dict):
            return {k: replace(v) for k, v in value.items()}
        if isinstance(value, list):
            return [replace(v) for v in value]
        return value

    params = replace(yaml.safe_load((source/'config/flyco.yaml').read_text()))
    tracking = params['/sam_tracking']['ros__parameters']
    tracking['segmentation_backend'] = os.environ.get('FLYCO_ISAAC_SEGMENTATION_BACKEND', tracking['segmentation_backend'])
    tracking['segmentation_endpoint'] = os.environ.get('FLYCO_ISAAC_SEGMENTATION_ENDPOINT', tracking['segmentation_endpoint'])
    for entry in params.values():
        entry['ros__parameters']['use_sim_time'] = True
    params['/px4ctrl']['ros__parameters'].update(
        mass=meta['mass_kg'], full_thrust=sensor_config['full_thrust'],
        hov_percent=meta['mass_kg']*9.81/sensor_config['full_thrust'])
    planner = params['/flyco_planner']['ros__parameters']
    if sensor_config.get('flight_controller') == 'px4-native':
        planner['flyco_planner.init_traj'] = False
    # kdline writes ../../output.txt relative to its working directory.
    # Keep both sampler inputs and outputs inside this simulation session.
    fps = runtime/'fps'
    fps_work = fps/'worker/build'
    fps_work.mkdir(parents=True, exist_ok=True)
    (fps_work/'kdline').unlink(missing_ok=True)
    (fps_work/'kdline').symlink_to(build/'kdline')
    solver = runtime/'solver'
    for folder in ('global', 'local'):
        (solver/folder).mkdir(parents=True, exist_ok=True)
    planner.update({
                    'hcplanner.global_par_file': str(solver/'global/global.par'),
                    'hcplanner.global_problem_file': str(solver/'global/global.tsp'),
                    'hcplanner.global_result_file': str(solver/'global/global.txt'),
                    'hcplanner.local_folder': str(solver/'local'),
                    'flyco_planner.init_path_file': str(runtime/'initial_waypts.yaml'),
                    'fps.cpu': str(fps_work), 'fps.input': str(fps/'input.txt'),
                    'fps.output': str(fps/'output.txt'),
                    'flyco_planner.sim': False,
                    'flyco_planner.odom_fps': 30.,
                    'sdf_map.map_size_x': 64., 'sdf_map.map_size_y': 48.,
                    'sdf_map.map_back_x': -13.,
                    'sdf_map.map_size_z': 12., 'sdf_map.ground_height': -.5,
                    'sdf_map.virtual_ceil_height': 7.,
                    'viewpoint_manager.viewpoints_distance': sensor_config['viewpoint_distance'],
                    **{'map_ros.'+k: float(sensor_config[k]) for k in ('fx', 'fy', 'cx', 'cy')}})
    params['/rgb_point_mapping_node']['ros__parameters'].update({
        'rgb_mapping.model': str(runtime/'coal_reference.ply'),
        'rgb_mapping.origin_x': -32., 'rgb_mapping.origin_y': -24.,
        'rgb_mapping.origin_z': -.5, 'rgb_mapping.map_size_x': 64.,
        'rgb_mapping.map_size_y': 48., 'rgb_mapping.map_size_z': 12.})
    params['/mesh_predict_node']['ros__parameters'].update({
        'camera_info_topic': '/isaac/camera/info', 'min_height': .25,
        'intrinsics_mode': 'camera_info', 'pose_alignment_mode': 'identity',
        'observation_voxel_size': sensor_config['observation_voxel_size']})
    params['/sam_tracking']['ros__parameters']['isaac.camera_offset'] = sensor_config['camera_offset']
    for name in ('sam_tracking', 'sam_tracking_node'):
        params['/'+name]['ros__parameters']['min_height'] = .25
    cluster = params['/sam_tracking_node']['ros__parameters']
    for key, value in list(cluster.items()):
        if key.endswith('_path') or key in ('save_path', 'load_path'):
            cluster[key] = str(runtime/'cluster'/Path(str(value)).name)
    cluster['save_path'] = cluster['load_path'] = str(runtime/'cluster')
    (runtime/'cluster').mkdir(exist_ok=True)
    config = runtime/'flyco_params.yaml'
    config.write_text(yaml.safe_dump(params, sort_keys=False))
    actions = [ExecuteProcess(cmd=['/usr/bin/python3', str(root/'bridge/ros_bridge.py')],
                              output='screen')]
    critical = actions[:]
    omitted = set()
    if sensor_config.get('flight_controller') in ('px4', 'px4-native'):
        omitted.add('px4ctrl')  # PX4 owns all position/attitude/rate control loops
    if sensor_config.get('flight_controller') == 'px4-native':
        if os.environ.get('FLYCO_ISAAC_EXTERNAL_NATIVE_MODE') != '1':
            mode = Node(package='flyco_real', executable='px4_bridge.py',
                parameters=[{'simulation': True, 'use_sim_time': True, 'odom_topic': '/isaac/odom',
                             'bounds_min': [-32., -24., -.5], 'bounds_max': [32., 24., 10.]}], output='screen')
            actions.append(mode)
            critical.append(mode)
        actions.append(Node(package='flyco_real', executable='task.py', name='ground_heartbeat',
                            parameters=[{'heartbeat': True}], output='screen'))
        critical.append(actions[-1])
    if os.environ.get('FLYCO_ISAAC_NO_PERCEPTION') == '1':
        omitted.update(('sam_tracking', 'tracking_prompt', 'sam_tracking_node', 'mesh_predict_node'))
    if os.environ.get('FLYCO_ISAAC_NO_UI') == '1':
        omitted.add('prompt_ui')
    # RViz uses the project display configuration.
    if os.environ.get('FLYCO_ISAAC_RVIZ') == '1':
        actions.append(ExecuteProcess(cmd=['bash', str(root/'tools/run_rviz.sh')],
                                      output='screen'))
    if 'sam_tracking' not in omitted:
        actions.append(ExecuteProcess(cmd=[str(flyco_runtime/'venv/bin/python'),
            str(root/'bridge/perception_image.py')], output='screen'))
    nodes = []
    for spec in yaml.safe_load((source/'config/nodes.yaml').read_text()):
        name = spec['name']
        if name in omitted:
            continue
        nodes.append(name)
        remaps = [tuple(pair) for pair in spec.get('remappings', [])]
        if name == 'sam_tracking':
            actions.append(ExecuteProcess(cmd=[str(flyco_runtime/'venv/bin/python'),
                str(root/'bridge/tracking_node.py'), '--ros-args', '-r', '__node:=sam_tracking',
                '--params-file', str(config)], output='screen'))
        else:
            executable = spec['executable']
            if spec.get('python'):
                actions.append(Node(package='flyco_ros2', executable=executable, name=name,
                    parameters=[str(config)], remappings=remaps,
                    arguments=['--ros-args', '--log-level', 'warn', '--log-level',
                               'flyco.task_controls:=info'], output='screen'))
            else:
                binary = build/executable
                if not binary.is_file():
                    raise FileNotFoundError('Run tools/setup_flyco.sh first: '+str(binary))
                command = [str(binary), '--ros-args', '-r', '__node:='+name,
                           '--params-file', str(config), '--log-level', 'warn',
                           '--log-level', 'flyco.task_controls:=info']
                for a, b in remaps:
                    command.extend(['-r', a+':='+b])
                actions.append(ExecuteProcess(cmd=command, output='screen'))
        if name != 'prompt_ui':
            critical.append(actions[-1])
    (runtime/'algorithm_nodes.json').write_text(json.dumps(nodes, indent=2))
    # A failed algorithm must stop the session; rendering alone is not success.
    for process in critical:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A FlyCo/bridge process exited'))])))
    return LaunchDescription(actions)
