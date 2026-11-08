"""Ground prediction and optional semantic/RViz processes; launch the UI separately."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    def path(value):
        # Keep the venv's bin/python path: resolving that symlink loses its environment.
        return Path(os.path.abspath(os.path.expanduser(os.path.expandvars(value))))

    root = path(arg('project_root'))
    config = path(arg('config'))
    settings = yaml.safe_load(config.read_text()).get('deployment', {})
    runtime = path(os.environ.get('FLYCO_NATIVE_ROOT', str(root / 'runtime/native')))
    predictor = path(arg('model_python') or str(
        Path(os.environ.get('FLYCO_NATIVE_VENV', str(runtime / 'predictor'))) / 'bin/python'))
    models = path(arg('models_dir') or os.environ.get('FLYCO_MODELS', str(runtime / 'models')))
    cache = path(arg('cache_dir') or str(runtime / 'torch_extensions'))
    cuda_setting = arg('cuda_home') or os.environ.get('CUDA_HOME', '')
    cuda = path(cuda_setting or settings.get('ground_cuda_home', '/usr/local/cuda-11.8'))
    local_cuda = root / '.local/deploy/cuda-root/usr/local/cuda-11.8'
    if not cuda_setting and not os.access(cuda / 'bin/nvcc', os.X_OK):
        cuda = local_cuda
    if not os.access(predictor, os.X_OK):
        raise FileNotFoundError('Prediction Python missing: %s; run bash tools/setup_ground_predictor.sh '
                               'from the project root, then set model_python:=/path/to/python' % predictor)
    for checkpoint in ('mesh/checkpoint.pt', 'mesh/snet-n3k-wonormal.pth'):
        if not (models / checkpoint).is_file():
            raise FileNotFoundError('Prediction checkpoint missing: ' + str(models / checkpoint))
    if not os.access(cuda / 'bin/nvcc', os.X_OK) or not (cuda / 'include/cuda_runtime_api.h').is_file():
        raise FileNotFoundError(
            'CUDA compiler or headers missing: %s; set cuda_home:=%s '
            '(install with bash tools/setup_ground_predictor.sh)' % (cuda, local_cuda))
    cache.mkdir(parents=True, exist_ok=True)
    # PyTorch's CUDA wheels supply library headers absent from the minimal toolkit.
    cuda_headers = sorted((predictor.parent.parent / 'lib').glob(
        'python*/site-packages/nvidia/*/include'))
    environment = {
        'FLYCO_MODELS': str(models),
        'MESH_PREDICT_MESH_CKPT': str(models / 'mesh/checkpoint.pt'),
        'MESH_PREDICT_NKSR_CKPT': str(models / 'mesh/snet-n3k-wonormal.pth'),
        'HF_HOME': str(models / 'huggingface'), 'TORCH_HOME': str(models / 'torch'),
        'TORCH_EXTENSIONS_DIR': str(cache), 'CUDA_HOME': str(cuda),
        'PATH': str(cuda / 'bin') + os.pathsep + os.environ.get('PATH', ''),
        'LD_LIBRARY_PATH': str(cuda / 'lib64') + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''),
        'CPATH': os.pathsep.join([str(p) for p in cuda_headers]
                                + [os.environ.get('CPATH', '')]).rstrip(os.pathsep),
        'MAX_JOBS': os.environ.get('MAX_JOBS', '2'),
        'PYTHONDONTWRITEBYTECODE': '1',
        'PYTHONNOUSERSITE': '1',
    }
    pointnet_extension = cache / '_ext/_ext.so'
    if pointnet_extension.is_file() and not os.environ.get('POINTNET2_OPS_EXTENSION_PATH'):
        environment['POINTNET2_OPS_EXTENSION_PATH'] = str(pointnet_extension)
    actions = [SetEnvironmentVariable(key, value) for key, value in environment.items()]
    if arg('start_semantic').lower() == 'true':
        semantic_root = path(arg('semantic_root') or str(root / 'third_party/SAM_AD_Perception'))
        semantic_python = path(arg('semantic_python') or str(semantic_root / '.venv/bin/python'))
        if not (semantic_root / 'sam_ad/server.py').is_file():
            raise FileNotFoundError('Semantic sources missing; set project_root or semantic_root: ' + str(semantic_root))
        if not os.access(semantic_python, os.X_OK):
            raise FileNotFoundError('Semantic Python missing: %s; set semantic_python:=/path/to/python3.12' % semantic_python)
        semantic = ExecuteProcess(
            cmd=[str(semantic_python), '-B', '-m', 'sam_ad.server', '--backend', 'A',
                 '--bind', str(settings.get('semantic_bind', '127.0.0.1')),
                 '--port', str(settings.get('semantic_port', 8851))],
            cwd=str(semantic_root), name='semantic_perception', output='screen')
        actions.append(RegisterEventHandler(OnProcessExit(target_action=semantic,
            on_exit=[EmitEvent(event=Shutdown(reason='Semantic perception service exited'))])))
        actions.append(semantic)
    share = Path(get_package_share_directory('flyco_real'))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/native.launch.py')),
        launch_arguments={'role': 'ground', 'config': str(config),
                          'python_executable': str(predictor),
                          'runtime_root': arg('runtime_root') or str(runtime / 'runs')}.items()))
    if arg('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
            arguments=['-d', str(share/'config/flyco_real.rviz')],
            parameters=[{'use_sim_time': False}], output='screen'))
    return actions


def generate_launch_description():
    root = os.environ.get('COAL_FLYCO_ROOT', str(Path.cwd()))
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=root),
        DeclareLaunchArgument('config', default_value=[LaunchConfiguration('project_root'), '/config/vehicle.local.yaml']),
        DeclareLaunchArgument('model_python', default_value=''),
        DeclareLaunchArgument('models_dir', default_value=''),
        DeclareLaunchArgument('semantic_root', default_value=''),
        DeclareLaunchArgument('semantic_python', default_value=''),
        DeclareLaunchArgument('cuda_home', default_value=''),
        DeclareLaunchArgument('cache_dir', default_value=''),
        DeclareLaunchArgument('runtime_root', default_value=''),
        DeclareLaunchArgument('start_semantic', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=build),
    ])
