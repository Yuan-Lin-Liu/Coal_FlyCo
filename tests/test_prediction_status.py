import ast
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'third_party/flyco_ros2/vendor/perception_predictor/src/mesh_predict/src/mesh_predict'


def load_function(path, name, namespace, class_name=None):
    tree = ast.parse(path.read_text())
    if class_name is not None:
        tree = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


@pytest.fixture
def diagnostics():
    namespace = {'MeshPredictOrchestrator': object}
    source = SOURCE / 'core/orchestrator.py'
    load_function(source, 'format_waiting_reason', namespace)
    load_function(source, 'format_prediction_status', namespace)
    load_function(source, 'is_ready_for_pc_prediction', namespace, 'MeshPredictOrchestrator')
    return SimpleNamespace(**namespace)


def orchestrator(points=0, images=0, camera_info=False, mode='camera_info'):
    return SimpleNamespace(
        state=SimpleNamespace(
            input_pointcloud=SimpleNamespace(points=[None] * points),
            image_conditions=[None] * images,
        ),
        camera_info_received=camera_info,
        intrinsics_mode=mode,
        observation_voxel_size=0.6,
        pc_prediction_stage='waiting_for_inputs',
        mesh_prediction_stage='waiting_for_pointcloud_prediction',
    )


def test_status_shows_all_missing_inputs_and_actual_accumulated_count(diagnostics):
    status = diagnostics.format_prediction_status(orchestrator(points=173, images=2))

    assert 'accumulated_points=173/512' in status
    assert 'image_conditions=2/3' in status
    assert 'camera_info=missing' in status
    assert 'voxel_size=0.600m' in status
    assert 'Waiting for image pose conditions' in status
    assert 'accumulated point cloud samples' in status
    assert 'camera parameters' in status


@pytest.mark.parametrize(
    'points,images,camera_info,mode,ready',
    [
        (511, 3, True, 'camera_info', False),
        (512, 2, True, 'camera_info', False),
        (512, 3, False, 'camera_info', False),
        (512, 3, True, 'camera_info', True),
        (512, 3, False, 'training_legacy', True),
    ],
)
def test_reported_readiness_matches_existing_prediction_gate(
    diagnostics, points, images, camera_info, mode, ready,
):
    instance = orchestrator(points, images, camera_info, mode)

    assert diagnostics.is_ready_for_pc_prediction(instance) is ready
    assert (diagnostics.format_waiting_reason(instance) == 'Prediction inputs ready') is ready


def test_periodic_log_includes_processing_stage_and_pending_messages(diagnostics):
    messages = []
    instance = orchestrator(points=684, images=8, camera_info=True)
    instance.pc_prediction_stage = 'pointcloud_inference'
    node = SimpleNamespace(
        orchestrator=instance,
        pending_point_messages=deque([object(), object()]),
        pending_img_pose_messages=deque([object()]),
        _mesh_frozen=False,
        get_logger=lambda: SimpleNamespace(info=messages.append),
    )
    callback = load_function(
        SOURCE / 'ros/node.py', 'prediction_status_timer_callback',
        {'format_prediction_status': diagnostics.format_prediction_status}, 'MeshPredictNode',
    )

    callback(node)

    assert len(messages) == 1
    assert 'accumulated_points=684/512' in messages[0]
    assert 'pc_stage=pointcloud_inference' in messages[0]
    assert 'mesh_stage=waiting_for_pointcloud_prediction' in messages[0]
    assert 'pending_cloud_messages=2/5' in messages[0]
    assert 'pending_image_messages=1' in messages[0]
    assert 'mesh_frozen=False' in messages[0]
