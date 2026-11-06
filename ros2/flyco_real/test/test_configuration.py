"""Installation/configuration and authenticated HTTP client boundary."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from flyco_real.bringup import prepare_parameters
from sam_tracking.providers.http_text_sam_provider import HTTPTextSAMProvider


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
