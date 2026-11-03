#!/usr/bin/env bash
# Use Kit Python without inheriting the host's ROS Python 3.10 environment.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_PATH="${ISAACSIM_PATH:-${ISAAC_ROOT:-}}"
if [[ -z "$ISAACSIM_PATH" ]]; then
  for candidate in "$ROOT/.local/isaacsim" /opt/isaacsim /usr/local/isaacsim "$HOME/isaacsim"; do
    if [[ -f "$candidate/python.sh" ]]; then
      ISAACSIM_PATH="$candidate"
      break
    fi
  done
fi
if [[ ! -x "$ISAACSIM_PATH/python.sh" ]]; then
  echo "[ERROR] Isaac Sim executable python.sh was not found." >&2
  echo "        Set ISAACSIM_PATH to the Isaac Sim 5.1 installation directory." >&2
  exit 1
fi
export ISAACSIM_PATH
export PYTHONNOUSERSITE=1
unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset PYTHONPATH PYTHONHOME CMAKE_PREFIX_PATH
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  IFS=: read -r -a runtime_libs <<< "$LD_LIBRARY_PATH"
  clean_libs=()
  for library in "${runtime_libs[@]}"; do
    if [[ -n "$library" && "$library" != /opt/ros/* ]]; then
      clean_libs+=("$library")
    fi
  done
  if [[ ${#clean_libs[@]} -gt 0 ]]; then
    LD_LIBRARY_PATH="$(IFS=:; echo "${clean_libs[*]}")"
    export LD_LIBRARY_PATH
  else
    unset LD_LIBRARY_PATH
  fi
fi
exec "$ISAACSIM_PATH/python.sh" "$@"
