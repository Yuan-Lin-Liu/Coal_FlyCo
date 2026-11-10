"""Convert saved RViz display settings into a native Foxglove layout."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import tempfile

import yaml


STANDARD_DISPLAYS = {'PointCloud2', 'Marker', 'MarkerArray', 'Path', 'Odometry',
                     'Pose', 'PoseArray', 'PoseWithCovariance', 'LaserScan', 'Map', 'TF'}


def _color(value, alpha=1.0):
    channels = [float(part.strip()) for part in str(value).split(';')]
    if len(channels) != 3 or not all(math.isfinite(c) for c in channels):
        raise ValueError('Expected an RViz RGB color: ' + str(value))
    values = [max(0, min(255, round(c))) for c in channels]
    values.append(max(0, min(255, round(float(alpha) * 255))))
    return '#' + ''.join(f'{c:02x}' for c in values)


def _topic(display):
    value = display.get('Topic', '')
    value = value.get('Value', '') if isinstance(value, dict) else value
    return str(value).strip()


def _mapping(value, name):
    if not isinstance(value, dict):
        raise ValueError(name + ' must be a mapping')
    return value


def _displays(displays, enabled=True, group=''):
    for display in displays:
        if not isinstance(display, dict):
            raise ValueError('Each RViz display must be a mapping')
        active = enabled and bool(display.get('Enabled', True))
        label = '/'.join(filter(None, (group, display.get('Name', ''))))
        if display.get('Class') == 'rviz_common/Group':
            yield from _displays(display.get('Displays', []), active, label)
        else:
            yield display, active, label


def _split_panels(panels, direction='column'):
    if len(panels) == 1:
        return panels[0]
    return {'direction': direction, 'first': panels[0],
            'second': _split_panels(panels[1:], direction),
            'splitPercentage': round(100 / len(panels), 2)}


def convert_rviz(document):
    """Return (importable layout, conversion report), without changing the input."""
    if not isinstance(document, dict) or not isinstance(document.get('Visualization Manager'), dict):
        raise ValueError('Missing RViz Visualization Manager')
    manager = document['Visualization Manager']
    options = _mapping(manager.get('Global Options', {}), 'Global Options')
    frame = str(options.get('Fixed Frame', 'world'))
    settings = {
        'fixedFrame': frame, 'followTf': frame, 'followMode': 'follow-none',
        'scene': {'backgroundColor': _color(options.get('Background Color', '28; 32; 40'))[:7]},
        'topics': {}, 'transforms': {}, 'layers': {},
    }
    views = _mapping(manager.get('Views', {}), 'Views')
    view = _mapping(views.get('Current', {}), 'Current view')
    focal = _mapping(view.get('Focal Point', {}), 'Focal Point')
    settings['cameraState'] = {
        'perspective': True, 'distance': float(view.get('Distance', 25)),
        'phi': 90 - math.degrees(float(view.get('Pitch', .8))),
        'thetaOffset': -math.degrees(float(view.get('Yaw', .9))) - 90,
        'target': [0, 0, 0], 'targetOrientation': [0, 0, 0, 1],
        'targetOffset': [float(focal.get(axis, 0)) for axis in ('X', 'Y', 'Z')],
        'fovy': 45, 'near': float(view.get('Near Clip Distance', .05)), 'far': 5000,
    }
    panels = {'3D!flyco': settings}
    report = {'fixed_frame': frame, 'topics': [], 'displays': [], 'unsupported': [],
              'odometry_adapters': []}
    images = []
    for display, active, label in _displays(manager.get('Displays', [])):
        class_name = str(display.get('Class', ''))
        kind = class_name.removeprefix('rviz_default_plugins/')
        topic = _topic(display)
        entry = {'name': label, 'class': class_name, 'topic': topic, 'enabled': active}
        report['displays'].append(entry)
        if kind == 'Grid':
            cell = float(display.get('Cell Size', 1))
            divisions = max(1, int(display.get('Plane Cell Count', 10)))
            offset = _mapping(display.get('Offset', {}), 'Grid Offset')
            key = 'grid-' + str(len(settings['layers']))
            settings['layers'][key] = {
                'visible': active, 'label': display.get('Name', 'Grid'),
                'instanceId': key, 'layerId': 'foxglove.Grid',
                'size': cell * divisions, 'divisions': divisions, 'lineWidth': 1,
                'color': _color(display.get('Color', '80; 85; 95'), display.get('Alpha', 1)),
                'position': [float(offset.get(axis, 0)) for axis in ('X', 'Y', 'Z')],
                'rotation': [0, 0, 0], 'order': len(settings['layers']),
            }
            continue
        if kind == 'Axes':
            axes_frame = display.get('Reference Frame', frame)
            if axes_frame == '<Fixed Frame>':
                axes_frame = frame
            settings['transforms'][axes_frame] = {'visible': active}
            continue
        if kind != 'Image' and kind not in STANDARD_DISPLAYS:
            report['unsupported'].append(entry)
            continue
        if not topic:
            continue
        report['topics'].append(topic)
        if kind == 'Image':
            if active and topic not in images:
                images.append(topic)
                panels[f'Image!flyco{len(images)}'] = {
                    'cameraTopic': topic,
                    'imageMode': {'imageTopic': topic},
                    'topics': {}, 'layers': {},
                }
            continue
        if kind == 'Odometry':
            target = '/foxglove/poses/' + topic.lstrip('/').replace('~', 'private_')
            adapter = {'source': topic, 'target': target}
            if adapter not in report['odometry_adapters']:
                report['odometry_adapters'].append(adapter)
            report['topics'].append(target)
            topic = target
        topic_settings = {'visible': active}
        if kind == 'PointCloud2' or kind == 'LaserScan':
            transformer = display.get('Color Transformer', 'FlatColor')
            if transformer in ('RGB8', 'RGBF32'):
                topic_settings.update(colorMode='rgb', colorField='rgb')
            elif transformer == 'FlatColor':
                topic_settings.update(colorMode='flat', flatColor=_color(
                    display.get('Color', '255; 255; 255'), display.get('Alpha', 1)))
            else:
                field = (display.get('Axis', 'z') if transformer == 'AxisColor'
                         else display.get('Channel Name', 'intensity'))
                if not isinstance(field, str):
                    raise ValueError('Point cloud color field must be a string')
                topic_settings.update(colorMode='colormap', colorMap='turbo',
                                      colorField=field.lower() if transformer == 'AxisColor' else field)
            topic_settings.update(pointSize=float(display.get('Size (Pixels)', 2)),
                                  decayTime=float(display.get('Decay Time', 0)))
        elif kind in ('Path', 'Odometry'):
            style = _mapping(display.get('Shape', {}), 'Odometry Shape') if kind == 'Odometry' else display
            if 'Color' in style:
                topic_settings['color'] = _color(style['Color'], style.get('Alpha', 1))
            if 'Line Width' in display:
                topic_settings['lineWidth'] = float(display['Line Width'])
        previous = settings['topics'].get(topic)
        if previous is None or (active and not previous['visible']):
            settings['topics'][topic] = topic_settings
    report['topics'] = sorted(set(report['topics']))
    image_ids = [key for key in panels if key.startswith('Image!')]
    arrangement = '3D!flyco'
    if image_ids:
        arrangement = {'direction': 'row', 'first': '3D!flyco',
                       'second': _split_panels(image_ids), 'splitPercentage': 72}
    return {
        'configById': panels, 'layout': arrangement, 'globalVariables': {},
        'userNodes': {}, 'playbackConfig': {'speed': 1},
    }, report


def topic_whitelist(topics):
    """Expose configured display topics and TF, without a catch-all expression."""
    names = sorted(set(topics) | {'/tf', '/tf_static', '/clock', '/rosout',
                                 '/rviz_to_foxglove/layout'})
    return ['^(?:' + '|'.join(re.escape(name) for name in names) + ')$']


def export_layout(rviz_path, output_path):
    source, target = Path(rviz_path).expanduser().resolve(), Path(output_path).expanduser().resolve()
    if source == target:
        raise ValueError('Layout output must differ from the RViz source')
    try:
        document = yaml.safe_load(source.read_text())
    except yaml.YAMLError as error:
        raise ValueError('Invalid RViz YAML: ' + str(error)) from error
    layout, report = convert_rviz(document)
    serialized = json.dumps(layout, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=target.parent,
                                         prefix=target.name + '.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(serialized)
        os.replace(temporary, target)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return layout, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('rviz', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    _, report = export_layout(args.rviz, args.output)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
