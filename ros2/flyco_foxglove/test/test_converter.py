import copy
import json
import math
from pathlib import Path
import sys

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE))

from flyco_foxglove.converter import convert_rviz, export_layout, topic_whitelist


def configuration(displays):
    return {'Visualization Manager': {
        'Global Options': {'Fixed Frame': 'world', 'Background Color': '28; 32; 40'},
        'Displays': displays,
    }}


def scene(layout):
    return layout['configById']['3D!flyco']


def test_saved_project_configuration_keeps_topics_and_default_visibility():
    source = yaml.safe_load((ROOT / 'ros2/flyco_real/config/flyco_real.rviz').read_text())
    original = copy.deepcopy(source)
    layout, report = convert_rviz(source)

    assert source == original
    assert scene(layout)['fixedFrame'] == 'world'
    assert scene(layout)['followTf'] == 'world'
    assert scene(layout)['topics']['/prediction/mesh_vis']['visible'] is True
    assert scene(layout)['topics']['/mesh_pred/pred_pc']['visible'] is False
    assert scene(layout)['topics']['/flyco/camera/fov']['visible'] is True
    assert scene(layout)['topics']['/traj_server/exec_traj']['visible'] is True
    assert scene(layout)['topics']['/foxglove/poses/flyco/odom']['visible'] is True
    assert '/flyco/odom' not in scene(layout)['topics']
    assert report['odometry_adapters'] == [
        {'source': '/flyco/odom', 'target': '/foxglove/poses/flyco/odom'}]
    images = [value['imageMode']['imageTopic'] for key, value in layout['configById'].items()
              if key.startswith('Image!')]
    assert images == ['/flyco/camera/rgb', '/tracking/mask/image']
    assert report['unsupported'] == []
    assert json.loads(json.dumps(layout)) == layout


def test_disabled_parent_group_hides_children_and_does_not_open_image_panel():
    layout, report = convert_rviz(configuration([{
        'Class': 'rviz_common/Group', 'Name': 'Hidden', 'Enabled': False,
        'Displays': [
            {'Class': 'rviz_default_plugins/PointCloud2', 'Enabled': True,
             'Topic': {'Value': '/cloud'}},
            {'Class': 'rviz_default_plugins/Image', 'Enabled': True,
             'Topic': {'Value': '/image'}},
        ],
    }]))

    assert scene(layout)['topics']['/cloud']['visible'] is False
    assert not any(key.startswith('Image!') for key in layout['configById'])
    assert '/image' in report['topics']


def test_orbit_view_keeps_the_camera_on_the_same_side_of_the_target():
    source = configuration([])
    source['Visualization Manager']['Views'] = {'Current': {
        'Distance': 55., 'Pitch': .8, 'Yaw': .9}}
    layout, _ = convert_rviz(source)
    camera = scene(layout)['cameraState']
    phi, theta = math.radians(camera['phi']), math.radians(camera['thetaOffset'])
    foxglove_offset = [-55 * math.sin(phi) * math.sin(theta),
                      -55 * math.sin(phi) * math.cos(theta), 55 * math.cos(phi)]
    rviz_offset = [55 * math.cos(.8) * math.cos(.9),
                   55 * math.cos(.8) * math.sin(.9), 55 * math.sin(.8)]
    assert foxglove_offset == pytest.approx(rviz_offset)


def test_duplicate_topic_retains_enabled_style_and_images_are_deduplicated():
    active = {'Class': 'rviz_default_plugins/PointCloud2', 'Enabled': True,
              'Topic': '/cloud', 'Color': '50; 255; 100', 'Alpha': .5,
              'Size (Pixels)': 3, 'Color Transformer': 'FlatColor'}
    hidden = {**active, 'Enabled': False, 'Color': '0; 0; 0'}
    image = {'Class': 'rviz_default_plugins/Image', 'Topic': '/rgb', 'Enabled': True}
    layout, _ = convert_rviz(configuration([active, hidden, image, image]))

    settings = scene(layout)['topics']['/cloud']
    assert settings['visible'] is True
    assert settings['flatColor'] == '#32ff6480'
    assert settings['pointSize'] == 3
    assert len([key for key in layout['configById'] if key.startswith('Image!')]) == 1


def test_unsupported_plugin_is_reported_without_inventing_a_visualization():
    layout, report = convert_rviz(configuration([
        {'Class': 'custom_plugins/Special', 'Name': 'Special', 'Topic': '/special'},
    ]))
    assert '/special' not in scene(layout)['topics']
    assert report['unsupported'][0]['class'] == 'custom_plugins/Special'


def test_whitelist_uses_literal_topic_names_and_includes_transforms():
    whitelist = topic_whitelist(['/sensors/cloud.v1'])
    import re
    pattern = re.compile(whitelist[0])
    assert pattern.fullmatch('/sensors/cloud.v1')
    assert not pattern.fullmatch('/sensors/cloudXv1')
    assert pattern.fullmatch('/tf')
    assert pattern.fullmatch('/tf_static')
    assert not pattern.fullmatch('/planning/pos_cmd')


@pytest.mark.parametrize('invalid', [
    'not a visualization configuration', '[unclosed',
    'Visualization Manager:\n  Global Options: null',
    'Visualization Manager:\n  Views: null',
    'Visualization Manager:\n  Views:\n    Current: null',
    'Visualization Manager:\n  Displays:\n  - Class: rviz_default_plugins/Grid\n    Offset: null',
    'Visualization Manager:\n  Displays:\n  - Class: rviz_default_plugins/PointCloud2\n'
    '    Topic: /cloud\n    Color Transformer: AxisColor\n    Axis: null',
])
def test_export_preserves_last_valid_output_when_input_is_invalid(tmp_path, invalid):
    source, output = tmp_path / 'input.rviz', tmp_path / 'layout.json'
    source.write_text(yaml.safe_dump(configuration([])))
    export_layout(source, output)
    before = output.read_bytes()
    source.write_text(invalid)
    with pytest.raises(ValueError):
        export_layout(source, output)
    assert output.read_bytes() == before


def test_export_refuses_to_overwrite_the_source_rviz_file(tmp_path):
    source = tmp_path / 'input.rviz'
    source.write_text(yaml.safe_dump(configuration([])))
    before = source.read_bytes()
    with pytest.raises(ValueError):
        export_layout(source, source)
    assert source.read_bytes() == before
