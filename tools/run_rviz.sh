#!/usr/bin/env bash
# Reuse the current session's ROS domain and Isaac display configuration.
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_ROOT/tools/flyco_env.sh"
if [[ "${FLYCO_ISAAC_FLIGHT_CONTROLLER:-}" == px4-native ]]; then
  export ROS_LOCALHOST_ONLY=0
fi
export ROS_LOG_DIR="${FLYCO_ISAAC_RUNTIME:-$PROJECT_ROOT/runtime}/rviz_logs"
mkdir -p "$ROS_LOG_DIR"
exec /opt/ros/humble/lib/rviz2/rviz2 -d "$PROJECT_ROOT/config/coal_warehouse.rviz" \
  --ros-args -r __node:=rvizvisualisation -p use_sim_time:=true "$@"
