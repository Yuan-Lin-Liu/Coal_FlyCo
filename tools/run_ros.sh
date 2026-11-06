#!/usr/bin/env bash
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$PROJECT_ROOT/tools/flyco_env.sh"
if [[ "${FLYCO_ISAAC_FLIGHT_CONTROLLER:-}" == px4-native ]]; then
  export ROS_LOCALHOST_ONLY=0
  if [[ -f "$PROJECT_ROOT/install/local_setup.bash" ]]; then
    source "$PROJECT_ROOT/install/local_setup.bash"
  elif [[ -f "$PROJECT_ROOT/runtime/native/ground/install/local_setup.bash" ]]; then
    # Compatibility with checkouts built before the standard workspace layout.
    source "$PROJECT_ROOT/runtime/native/ground/install/local_setup.bash"
  else
    echo 'Build the workspace first: source /opt/ros/humble/setup.bash && colcon build' >&2
    exit 1
  fi
  if [[ -n "${FLYCO_PX4_MSGS_SETUP:-}" ]]; then
    source "$FLYCO_PX4_MSGS_SETUP"
  fi
fi
export ROS_LOG_DIR="$FLYCO_ISAAC_RUNTIME/ros_logs"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch "$PROJECT_ROOT/bridge/flyco.launch.py"
