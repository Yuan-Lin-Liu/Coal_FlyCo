"""Resolve a named single-pile scene and its matching runtime settings."""
from copy import deepcopy
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent


def scene_catalog():
    return yaml.safe_load((ROOT/'config/scenes.yaml').read_text())['scenes']


def _merge(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = deepcopy(value)


def resolve_scene(selection='4'):
    key = str(selection)
    catalog = scene_catalog()
    config = yaml.safe_load((ROOT/'config/coal_warehouse.yaml').read_text())
    if key in catalog:
        entry = catalog[key]
        _merge(config, entry.get('overrides', {}))
        profile = {'id': key, 'name': entry['name'],
                   'scene': ROOT/entry['scene'], 'metadata': ROOT/entry['metadata'],
                   'waypoints': ROOT/entry['waypoints']}
    else:
        path = Path(key).expanduser()
        if path.suffix.lower() not in ('.usd', '.usda', '.usdc'):
            raise ValueError('Unknown scene '+key+'; choose 4, 6, or a USD file path.')
        profile = {'id': 'custom', 'name': path.stem, 'scene': path.resolve(),
                   'metadata': None, 'waypoints': ROOT/'config/initial_waypts.yaml'}
    config.update(scene_id=profile['id'], scene_name=profile['name'])
    return dict(profile, config=config)
