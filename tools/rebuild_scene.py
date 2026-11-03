"""Rebuild selected single-pile scenes, preserving their previous USD files."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

if __name__ == '__main__':
    from scene_profiles import resolve_scene, scene_catalog

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', '--pile', default='4', choices=[*scene_catalog(), 'all'])
    args = parser.parse_args()
    sys.argv = [sys.argv[0]]
    from isaacsim import SimulationApp

    app = SimulationApp({'headless': True, 'limit_cpu_threads': 8,
                         'multi_gpu': False, 'fast_shutdown': True})
    try:
        from pxr import Usd
        import yaml
        from scene_builder import build_scene
        from scene_storage import save_scene

        selections = list(scene_catalog()) if args.scene == 'all' else [args.scene]
        for selection in selections:
            profile = resolve_scene(selection)
            stage = Usd.Stage.CreateInMemory()
            metadata = build_scene(stage, scene_config=profile['config'])
            metadata.update(scene_id=profile['id'], scene_name=profile['name'])
            metadata['initial_waypoints'] = yaml.safe_load(
                profile['waypoints'].read_text())['path'][0]['waypoints']
            saved = save_scene(stage, profile['scene'], stopped=True)
            profile['metadata'].write_text(json.dumps(metadata, indent=2)+'\n')
            print(json.dumps({'scene_id': profile['id'], 'scene': saved,
                'pile': metadata['piles'][0], 'spawn': metadata['start_position']}, indent=2), flush=True)
    finally:
        app.close()
