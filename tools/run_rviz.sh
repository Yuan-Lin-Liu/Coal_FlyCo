#!/usr/bin/env bash
# Reuse the current session's ROS domain and Isaac display configuration.
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_ROOT/tools/flyco_env.sh"
export ROS_LOG_DIR="${FLYCO_ISAAC_RUNTIME:-$PROJECT_ROOT/runtime}/rviz_logs"
mkdir -p "$ROS_LOG_DIR"
exec /opt/ros/humble/lib/rviz2/rviz2 -d "$PROJECT_ROOT/config/coal_warehouse.rviz" \
  --ros-args -r __node:=rvizvisualisation -p use_sim_time:=true "$@"
