"""Register the selected FlyCo executables and assets in a local ament prefix."""
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'third_party/flyco_ros2'
RUNTIME = Path(os.environ.get('FLYCO_RUNTIME', ROOT/'.local/flyco'))
PREFIX = RUNTIME/'install'
marker = PREFIX/'share/ament_index/resource_index/packages/flyco_ros2'
marker.parent.mkdir(parents=True, exist_ok=True)
marker.touch()
share = PREFIX/'share/flyco_ros2'
share.unlink(missing_ok=True)
share.symlink_to(os.path.relpath(SOURCE, share.parent), target_is_directory=True)
scripts = {
    'prompt_node.py': 'vendor/perception_predictor/src/sam_tracking/nodes/prompt_node.py',
    'prompt_ui_cv_node.py': 'vendor/perception_predictor/src/sam_tracking/nodes/prompt_ui_cv_node.py',
    'mesh_predict_node.py': 'vendor/perception_predictor/src/mesh_predict/scripts/mesh_predict_node.py',
}
destination = PREFIX/'lib/flyco_ros2'
destination.mkdir(parents=True, exist_ok=True)
for name, relative in scripts.items():
    entry = destination/name
    entry.unlink(missing_ok=True)
    entry.symlink_to(os.path.relpath(SOURCE/relative, destination))
for name in ('flyco_planner', 'traj_server', 'px4ctrl_node', 'joy_cmd',
             'rgb_point_mapping_node', 'sam_tracking_node'):
    entry = destination/name
    entry.unlink(missing_ok=True)
    entry.symlink_to(os.path.relpath(RUNTIME/'build'/name, destination))
print('Registered project-local FlyCo runtime:', PREFIX)
