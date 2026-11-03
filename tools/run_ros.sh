#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_ROOT/tools/flyco_env.sh"
export ROS_LOG_DIR="$FLYCO_ISAAC_RUNTIME/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch "$PROJECT_ROOT/bridge/flyco.launch.py"
