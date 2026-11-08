"""Installation/configuration and authenticated HTTP client boundary."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import hashlib
from pathlib import Path
import threading
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from flyco_real.bringup import prepare_parameters
import yaml
from sam_tracking.providers.http_text_sam_provider import HTTPTextSAMProvider


def test_installed_px4_definitions_match_provided_firmware_snapshot():
    share = Path(get_package_share_directory('px4_msgs'))
    source = json.loads((share / 'SOURCE.json').read_text())
    assert source['commit'] == '0d092a91f0c0417ab888b4bc37331484e6d46b9d'
    for name, expected in source['definitions_sha256'].items():
        relative = Path(name)
        installed = share / relative.parts[0] / relative.name
        assert hashlib.sha256(installed.read_bytes()).hexdigest() == expected, name


def test_installed_configuration_is_hardware_only(tmp_path):
    share = get_package_share_directory('flyco_ros2')
    libexec = Path(get_package_prefix('flyco_ros2')) / 'lib/flyco_ros2'
    hardware = Path(get_package_share_directory('flyco_real')) / 'config/hardware.yaml'
    params, run, file = prepare_parameters(share, libexec, hardware, tmp_path)
    assert file.is_file() and run.parent == tmp_path
    assert all(value['use_sim_time'] is False for value in params.values())
    assert not params['flyco_planner']['sdf_map.airsim_flag']
    assert not params['flyco_planner']['map_ros.dp']
    assert params['flyco_planner']['flyco_planner.initial_vertical_relative']
    assert params['sam_tracking']['pc_topic'] == '/flyco/lidar/world'
    assert params['mesh_predict_node']['pose_alignment_mode'] == 'identity'
    assert params['mesh_predict_node']['intrinsics_mode'] == 'camera_info'
    assert (run / 'fps/worker/build/kdline').resolve() == (libexec / 'kdline').resolve()
    assert '$(' not in file.read_text()
    import yaml
    predictor = params['mesh_predict_node']
    assert predictor['camera_param'] == ''
    assert predictor['profile_config_path'] == ''
    assert yaml.safe_load(Path(predictor['runtime_config_path']).read_text())['runtime']['camera_param_default'] is None


def test_legacy_odin_profile_uses_zero_mount_and_native_camera(tmp_path):
    profile = yaml.safe_load((Path(get_package_share_directory('flyco_real')) / 'config/odin.yaml').read_text())
    profile['sensor_bridge'] = profile.pop('perception_bridge')
    profile['sensor_bridge']['ros__parameters']['odom_topic'] = '/odin1/odometry_highfreq'
    profile['odin_adapter'] = {'ros__parameters': {'body_to_imu_translation': [.2, 0., 0.]}}
    profile['odin']['register_keys']['send_odom_baselink_tf'] = 0
    profile['sensor_bridge']['ros__parameters']['camera_optical_frame'] = 'odin_camera_optical_frame'
    path = tmp_path / 'old_vehicle.yaml'
    path.write_text(yaml.safe_dump(profile))
    params, _, output = prepare_parameters(get_package_share_directory('flyco_ros2'),
        Path(get_package_prefix('flyco_ros2')) / 'lib/flyco_ros2', path, tmp_path / 'runtime')
    assert 'odin_adapter' not in params
    assert 'body_to_imu' not in output.read_text()
    assert 'sensor_bridge' not in params
    assert 'odometry_topic' not in params['generic_odometry_node']  # New forwarder YAML owns this.
    assert params['generic_odometry_node']['extrinsic_translation'] == [0., 0., 0.]
    assert params['perception_bridge']['input_camera_frame'] == ''
    assert params['perception_bridge']['image_topic'] == '/odin1/image'
    assert params['perception_bridge']['odin_native_driver']
    assert not params['perception_bridge']['image_is_rectified']
    assert params['perception_bridge']['camera_optical_frame'] == 'camera_0'
    assert params['sam_tracking']['camera_optical_frame'] == 'camera_0'
    assert yaml.safe_load(path.read_text()) == profile


def test_odin_launch_keeps_driver_tf_with_legacy_control(tmp_path):
    import importlib.util
    from launch import LaunchContext
    from launch_ros.actions import Node
    share = Path(get_package_share_directory('flyco_real'))
    profile = yaml.safe_load((share / 'config/odin.yaml').read_text())
    profile['odin']['register_keys']['send_odom_baselink_tf'] = 0
    path = tmp_path / 'vehicle.yaml'
    path.write_text(yaml.safe_dump(profile))
    spec = importlib.util.spec_from_file_location('odin_hardware_launch', share / 'launch/hardware.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(config=str(path), role='sensors',
        runtime_root=str(tmp_path / 'runtime'), start_driver='true',
        odometry_config=str(Path(get_package_share_directory('ros2_px4_odometry')) / 'config/odin_config.yaml'))
    actions = module.build(context)
    executables = [action.node_executable for action in actions if isinstance(action, Node)]
    assert executables == ['host_sdk_sample', 'odom_to_px4_node', 'perception_bridge.py', 'camera_fov.py']
    control, = (tmp_path / 'runtime').glob('native-*/odin_control.yaml')
    assert yaml.safe_load(control.read_text())['register_keys']['send_odom_baselink_tf'] == 1
    assert yaml.safe_load(control.read_text())['register_keys']['sendrgbundistort'] == 0
    assert yaml.safe_load(control.read_text())['register_keys']['use_host_ros_time'] == 1


def test_http_remote_authorization_header():
    received = []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def do_GET(self):
            received.append(self.headers.get('Authorization'))
            data = json.dumps({'protocol': 'flyco-text-sam-v1', 'service_revision': '20261026-r3',
                               'backend': 'A', 'status': 'ready'}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        provider = HTTPTextSAMProvider('http://127.0.0.1:' + str(server.server_port), 'A',
                                       api_token='a-private-test-token-only')
        assert received == ['Bearer a-private-test-token-only']
        provider._disconnect()
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
