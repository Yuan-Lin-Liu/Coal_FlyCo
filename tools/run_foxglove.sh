#!/usr/bin/env bash
# Independent visualization; run alongside the existing Odin/FlyCo/RViz processes.
set -eo pipefail
FOXGLOVE_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$FOXGLOVE_PROJECT_ROOT/install/flyco_foxglove/share/flyco_foxglove/package.xml" ]]; then
  echo 'Build the visualization package first:' >&2
  echo '  colcon build --packages-select flyco_foxglove --symlink-install' >&2
  exit 1
fi
source "$FOXGLOVE_PROJECT_ROOT/install/setup.bash"
export COAL_FLYCO_ROOT="$FOXGLOVE_PROJECT_ROOT"
export ROS_DOMAIN_ID="${FOXGLOVE_DOMAIN:-${ROS_DOMAIN_ID:-66}}"
export ROS_LOG_DIR="$FOXGLOVE_PROJECT_ROOT/runtime/foxglove/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec ros2 launch flyco_foxglove foxglove.launch.py \
  domain:="$ROS_DOMAIN_ID" "$@"
