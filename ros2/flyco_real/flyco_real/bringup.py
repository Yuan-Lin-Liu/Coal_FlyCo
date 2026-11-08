"""Resolve standard installed packages; produce per-run writable solver resources."""
from pathlib import Path
import re
import tempfile
import yaml


def prepare_parameters(flyco_share, libexec, hardware_file, runtime_root):
    share, binaries = Path(flyco_share), Path(libexec)
    raw = yaml.safe_load((share / 'config/flyco.yaml').read_text())
    def resolve(value):
        if isinstance(value, str):
            value = re.sub(r'\$\(find-pkg-share flyco_ros2\)', str(share), value)
            value = re.sub(r'\$\(var [\w_]+\)', '', value)
            return value
        if isinstance(value, dict):
            return {k: resolve(v) for k, v in value.items()}
        if isinstance(value, list):
            return [resolve(v) for v in value]
        return value
    params = {key.lstrip('/'): resolve(value)['ros__parameters'] for key, value in raw.items()}
    for entry in params.values():
        entry['use_sim_time'] = False
    params['sam_tracking'].update({
        'odom_topic': '/flyco/odom', 'pc_topic': '/flyco/lidar/world',
        'img_topic': '/flyco/camera/rgb', 'camera_info_topic': '/flyco/camera/info',
        'gimbal_topic': '', 'airsim_camera_pose_enabled': False,
        'airsim_camera_optical_frame': 'camera_optical_frame',
    })
    params['sam_tracking_node'].update(all_point_topic='/flyco/lidar/world')
    params['flyco_planner'].update({
        'sdf_map.airsim_flag': False, 'map_ros.airsim_flag': False,
        'map_ros.dp': False, 'map_ros.dgp': False, 'map_ros.cp': False, 'map_ros.ctep': True,
        'flyco_planner.sim': False, 'flyco_planner.odom_fps': 30.,
        'flyco_planner.initial_vertical_relative': True,
        'flyco_planner.init_traj': False,
    })
    params['mesh_predict_node'].update({
        'camera_info_topic': '/flyco/camera/info', 'intrinsics_mode': 'camera_info',
        'pose_alignment_mode': 'identity', 'profile_config_path': '', 'camera_param': '',
    })
    overrides = yaml.safe_load(Path(hardware_file).read_text())
    for key, value in overrides.items():
        params.setdefault(key.lstrip('/'), {}).update(value.get('ros__parameters', {}))
    prefix = overrides.get('deployment', {}).get('px4_topic_prefix', '/fmu')
    params['px4_bridge']['px4_topic_prefix'] = prefix
    # Existing vehicle files may still use the removed sensor_bridge section.
    # Keep its camera/cloud settings. The new odin_config.yaml owns the
    # odometry input, rate and variance policy.
    legacy = params.pop('sensor_bridge', {})
    perception = {**legacy, **params.get('perception_bridge', {})}
    params['perception_bridge'] = perception
    forward = params.setdefault('generic_odometry_node', {})
    forward.setdefault('output_frame', perception.get('world_frame', 'world'))
    forward.setdefault('output_child_frame', perception.get('body_frame', 'base_link'))
    forward.setdefault('odometry_output_topic', '/flyco/odom')
    if not prefix.rstrip('/').endswith('/fmu'):
        raise ValueError('px4_topic_prefix must end in /fmu')
    forward.setdefault('topic_namespace_prefix', prefix.rstrip('/')[:-3])
    from .geometry import transform_matrix
    alignment = transform_matrix(perception.get('world_from_lio_translation', [0., 0., 0.]),
                                 perception.get('world_from_lio_quaternion', [0., 0., 0., 1.]))
    forward.setdefault('extrinsic_translation', alignment[:3, 3].tolist())
    forward.setdefault('extrinsic_rotation_matrix', alignment[:3, :3].reshape(-1).tolist())
    if overrides.get('deployment', {}).get('profile') == 'odin':
        # Odin's IMU origin is the vehicle origin: no mounting TF or lever arm.
        # Normalize older local profiles too, without rewriting vehicle settings.
        perception.update(
            odin_native_driver=True, image_topic='/odin1/image',
            image_is_rectified=False, input_camera_frame='',
            camera_optical_frame='camera_0')
    # Obsolete MAVROS endpoint keys must not redirect the native DDS provider.
    params['px4_bridge'].pop('mavros_ns', None)
    for key in ('odom_topic', 'odom_publish_rate', 'publish_external_vision', 'external_vision_topic',
                'external_vision_frame', 'pose_variances', 'odometry_is_body_frame', 'lio_child_frame',
                'lio_twist_frame', 'px4_topic_prefix'):
        perception.pop(key, None)
    # Only installed hardware-role nodes get parameters; legacy simulation
    # controllers and ground-truth RGB mapping are deliberately absent.
    names = {'flyco_planner', 'traj_server', 'sam_tracking', 'sam_tracking_node',
             'tracking_prompt', 'prompt_ui', 'mesh_predict_node', 'perception_bridge',
             'generic_odometry_node', 'px4_bridge'}
    params = {name: values for name, values in params.items() if name in names}
    for values in params.values():
        values['use_sim_time'] = False
    if perception.get('world_frame', 'world') != 'world':
        raise ValueError('The planner uses world as its map frame; configure world_from_lio instead')
    camera_frame = perception['camera_optical_frame']
    params['sam_tracking']['camera_optical_frame'] = camera_frame
    params['sam_tracking']['airsim_camera_optical_frame'] = camera_frame
    runtime_root = Path(runtime_root).expanduser()
    runtime_root.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix='native-', dir=runtime_root))
    fps = run / 'fps'
    worker = fps / 'worker/build'
    worker.mkdir(parents=True)
    (worker / 'kdline').symlink_to(binaries / 'kdline')
    for directory in ('solver/global', 'solver/local', 'cluster'):
        (run / directory).mkdir(parents=True)
    planner = params['flyco_planner']
    planner.update({
        'hcplanner.global_solver': str(binaries),
        'hcplanner.global_par_file': str(run / 'solver/global/global.par'),
        'hcplanner.global_problem_file': str(run / 'solver/global/global.tsp'),
        'hcplanner.global_result_file': str(run / 'solver/global/global.txt'),
        'hcplanner.local_folder': str(run / 'solver/local'),
        'fps.cpu': str(worker), 'fps.input': str(fps / 'input.txt'), 'fps.output': str(fps / 'output.txt'),
    })
    cluster = params['sam_tracking_node']
    for key in list(cluster):
        if key.endswith('_path'):
            cluster[key] = str(run / 'cluster' / Path(str(cluster[key])).name)
    cluster['save_path'] = cluster['load_path'] = str(run / 'cluster')
    # A native runtime has no simulator camera fallback, even before the first
    # CameraInfo arrives. Model/checkpoint resources retain their existing ABI.
    mesh = share / 'vendor/perception_predictor/src/mesh_predict'
    mesh_params = params['mesh_predict_node']
    native_runtime = yaml.safe_load(Path(mesh_params['runtime_config_path']).read_text())
    native_runtime['runtime']['camera_param_default'] = None
    native_runtime['runtime']['model_config_path'] = str(mesh / native_runtime['runtime']['model_config_path'])
    runtime_file = run / 'prediction_runtime.yaml'
    runtime_file.write_text(yaml.safe_dump(native_runtime, sort_keys=False))
    mesh_params['runtime_config_path'] = str(runtime_file)
    output = run / 'parameters.yaml'
    output.write_text(yaml.safe_dump({name: {'ros__parameters': entry} for name, entry in params.items()},
                                     allow_unicode=True, sort_keys=False))
    return params, run, output
