"""Ground image segmentation launches independently of the prediction stack."""
import importlib.util
from pathlib import Path

import pytest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[3]


def load(name, monkeypatch):
    path = ROOT / 'ros2/flyco_real/launch' / name
    spec = importlib.util.spec_from_file_location('segmentation_launch_test_' + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module, 'get_package_share_directory'):
        monkeypatch.setattr(module, 'get_package_share_directory',
                            lambda package: str(ROOT / 'ros2/flyco_real'))
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    context.launch_configurations.update(overrides)
    return context


def test_ground_only_mode_does_not_require_predictor_config_or_checkpoints(monkeypatch):
    module = load('ground.launch.py', monkeypatch)
    context = context_for(module, segmentation_only='true', config='/no/vehicle/config.yaml',
                          model_python='/no/predictor/python', project_root=str(ROOT))
    actions = module.build(context)
    assert len(actions) == 1
    arguments = dict(actions[0].launch_arguments)
    assert arguments['project_root'] == str(ROOT)
    assert arguments['rviz'] == context.launch_configurations['rviz']
    child = load('segmentation.launch.py', monkeypatch)
    # Includes inherit parent arguments: a vehicle config and predictor Python
    # must not override the dedicated segmentation config and uv interpreter.
    child_context = context_for(child)
    child_context.launch_configurations.update(context.launch_configurations)
    child_context.launch_configurations.update(arguments)
    settings = child.resolved_settings(child_context)
    assert settings['image_topic'] == '/odin1/image'
    assert settings['model_python'] == str(ROOT / '.local/deploy/semantic/bin/python')


def test_image_topic_override_and_uv_python_are_preserved(monkeypatch):
    module = load('segmentation.launch.py', monkeypatch)
    context = context_for(module, project_root=str(ROOT), image_topic='/flyco/camera/rgb')
    settings = module.resolved_settings(context)
    assert settings['image_topic'] == '/flyco/camera/rgb'
    assert settings['model_python'] == str(ROOT / '.local/deploy/semantic/bin/python')
    assert settings['source_dir'] == str(ROOT / 'third_party/SAM_AD_Perception')


def test_headless_image_segmentation_does_not_start_models_via_http_or_prediction(monkeypatch):
    module = load('segmentation.launch.py', monkeypatch)
    context = context_for(module, project_root=str(ROOT), ui='false', rviz='false')
    actions = module.build(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_executable == 'semantic_image.py'


def test_segmentation_ui_receives_the_selected_text_topic(monkeypatch):
    module = load('segmentation.launch.py', monkeypatch)
    context = context_for(module, project_root=str(ROOT), ui='true', rviz='false',
                          text_topic='/task/query', image_topic='/flyco/camera/rgb')
    includes = [action for action in module.build(context) if isinstance(action, IncludeLaunchDescription)]
    assert len(includes) == 1
    arguments = dict(includes[0].launch_arguments)
    assert arguments['image_topic'] == '/flyco/camera/rgb'
    assert arguments['text_topic'] == '/task/query'
    prompt = load('prompt.launch.py', monkeypatch)
    ui_context = context_for(prompt, **arguments)
    ui_node = prompt.build(ui_context)[0]
    ui_node._perform_substitutions(ui_context)
    assert ('/query_text', '/task/query') in ui_node.expanded_remapping_rules


@pytest.mark.parametrize('text', ['', 'false', '[chair]'])
def test_initial_prompt_stays_a_string_when_ros_evaluates_parameters(monkeypatch, text):
    module = load('segmentation.launch.py', monkeypatch)
    context = context_for(module, project_root=str(ROOT), text=text, ui='false', rviz='false')
    node = next(action for action in module.build(context) if isinstance(action, Node))
    parameters = evaluate_parameters(context, node._Node__parameters)[0]
    assert type(parameters['initial_text']) is str
    assert parameters['initial_text'] == text
