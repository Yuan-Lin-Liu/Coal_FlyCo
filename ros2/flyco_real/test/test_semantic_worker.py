"""Exercise a real subprocess; replace only the neural model boundary."""
import base64
import importlib
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'ros2/flyco_real'))


@pytest.fixture
def model_workspace(tmp_path):
    package = tmp_path / 'runtime_api'
    package.mkdir()
    (package / '__init__.py').write_text('')
    (package / 'models.py').write_text('''
from types import SimpleNamespace
import os
import numpy as np
class Pipeline:
    def process(self, rgb, index, total, command=None):
        print('model diagnostic on stdout')
        os.write(1, b'native model diagnostic on stdout\\n')
        if command is not None and command.text == 'fail':
            raise ValueError('controlled inference failure')
        mask = np.zeros(rgb.shape[:2], bool)
        mask[0, 1] = True
        return {'masks': {1: mask}, 'phase': 'TRACKING'}
def build_models(backend, workspace, app_root):
    print('model startup diagnostic on stdout')
    os.write(1, b'native startup diagnostic on stdout\\n')
    return Pipeline, lambda text: SimpleNamespace(quantity='one', text=text), None, None, {'backend': 'A', 'device': 'test', 'precision': 'FP32'}
''')
    return tmp_path


def make_client(workspace):
    module = importlib.import_module('flyco_real.semantic_ipc')
    # Both actual uv interpreters participate; no GPU model/checkpoint is loaded.
    return module.WorkerClient(
        str(ROOT / '.local/deploy/semantic/bin/python'), str(workspace))


def request(sequence=1, text='chair', prompt_id=1):
    ok, image = cv2.imencode('.png', np.full((3, 4, 3), 20, np.uint8))
    assert ok
    return {'kind': 'frame', 'id': sequence, 'prompt_id': prompt_id, 'text': text,
            'header': {'sec': 10, 'nanosec': 20, 'frame_id': 'camera_0'},
            'image_png': base64.b64encode(image).decode('ascii')}


def test_worker_preserves_frame_identity_and_is_not_confused_by_model_logging(model_workspace):
    client = make_client(model_workspace)
    try:
        ready = client.wait_ready(timeout=30)
        assert ready['kind'] == 'ready'
        result = client.segment(request(), timeout=30)
        assert result['id'] == 1
        assert result['prompt_id'] == 1
        assert result['header'] == {'sec': 10, 'nanosec': 20, 'frame_id': 'camera_0'}
        mask = cv2.imdecode(np.frombuffer(base64.b64decode(result['mask_png']), np.uint8), 0)
        assert mask.shape == (3, 4)
        assert np.count_nonzero(mask) == 1
        assert mask[0, 1] == 255
    finally:
        client.close()
    assert client.process.poll() is not None


def test_worker_reports_inference_errors_and_accepts_a_new_target(model_workspace):
    client = make_client(model_workspace)
    try:
        client.wait_ready(timeout=30)
        with pytest.raises(RuntimeError, match='controlled inference failure'):
            client.segment(request(text='fail'), timeout=30)
        result = client.segment(request(sequence=2, text='bag', prompt_id=2), timeout=30)
        assert result['kind'] == 'result'
        assert result['prompt_id'] == 2
    finally:
        client.close()


def test_worker_environment_does_not_inherit_ros_or_predictor_libraries(monkeypatch):
    monkeypatch.setenv('PYTHONPATH', '/a/ros/python/path')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/a/predictor/cuda/path')
    monkeypatch.setenv('CUDA_HOME', '/cuda11')
    monkeypatch.setenv('POINTNET2_OPS_EXTENSION_PATH', '/a/predictor/extension.so')
    env = importlib.import_module('flyco_real.semantic_ipc').worker_environment()
    assert 'PYTHONPATH' not in env
    assert 'LD_LIBRARY_PATH' not in env
    assert 'CUDA_HOME' not in env
    assert 'POINTNET2_OPS_EXTENSION_PATH' not in env
    assert env['SAM_ENCODER_BACKEND'] == 'torch'
