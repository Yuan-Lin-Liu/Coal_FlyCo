#!/usr/bin/env bash
# Build only this project's ROS interfaces and selected FlyCo algorithms.
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"
source "$PROJECT_ROOT/tools/flyco_env.sh"
# The runtime resource prefix is registered after the native build; only ROS
# and the interface overlay are needed while colcon generates message types.
export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH#"$FLYCO_RUNTIME/install:"}"
if [[ ! -x "$FLYCO_PYTHON" ]]; then
  echo 'Missing project-local FlyCo environment or native dependencies in .local/flyco.' >&2
  exit 1
fi
# This compatibility build has its own layout and must not load the root
# workspace's colcon_defaults.yaml into existing .local interface caches.
(
  cd "$FLYCO_RUNTIME"
  /usr/bin/colcon --log-base "$FLYCO_RUNTIME/interfaces/log" build \
    --base-paths "$PROJECT_ROOT/third_party/ros_interfaces" \
    --build-base "$FLYCO_RUNTIME/interfaces/build" \
    --install-base "$FLYCO_RUNTIME/interfaces/install" --merge-install \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
)
source "$FLYCO_RUNTIME/interfaces/install/local_setup.bash"
export CMAKE_PREFIX_PATH="$FLYCO_RUNTIME/deps/root/usr:${CMAKE_PREFIX_PATH:-}"
/usr/bin/cmake -S "$FLYCO_SOURCE" -B "$FLYCO_RUNTIME/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE=-O1 \
  -DFLYCO_BUILD_ALGORITHMS=ON -DFLYCO_BUILD_LEGACY_SIM_CONTROL=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_CXX_COMPILER=/usr/bin/c++ -DCMAKE_C_COMPILER=/usr/bin/cc
/usr/bin/cmake --build "$FLYCO_RUNTIME/build" --target \
  flyco_planner traj_server px4ctrl_node joy_cmd rgb_point_mapping_node sam_tracking_node kdline \
  -j "${BUILD_JOBS:-4}"
/usr/bin/python3 "$PROJECT_ROOT/tools/install_flyco_runtime.py"
