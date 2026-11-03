#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PEGASUS_EXT="${1:-${PEGASUS_EXT:-$PROJECT_ROOT/.local/PegasusSimulator/extensions/pegasus.simulator}}"
PATCH="$PROJECT_ROOT/patches/pegasus-5.1.0-local-changes.patch"

if [[ ! -f "$PATCH" ]]; then
  echo "[ERROR] Patch file not found: $PATCH" >&2
  exit 1
fi
if [[ ! -f "$PEGASUS_EXT/pegasus/simulator/params.py" ]]; then
  echo "[ERROR] Pegasus extension not found at: $PEGASUS_EXT" >&2
  echo "        Pass .../PegasusSimulator/extensions/pegasus.simulator as the first argument." >&2
  exit 1
fi
PEGASUS_EXT="$(cd -- "$PEGASUS_EXT" && pwd)"
# git apply otherwise discovers a parent checkout and silently skips these
# extension-relative paths when Pegasus came from a zip inside this project.
export GIT_CEILING_DIRECTORIES="$(dirname -- "$PEGASUS_EXT")"
if git -C "$PEGASUS_EXT" apply --check -p1 --whitespace=nowarn "$PATCH" >/dev/null 2>&1; then
  git -C "$PEGASUS_EXT" apply -p1 --whitespace=nowarn "$PATCH"
  grep -q 'skip_check=True' "$PEGASUS_EXT/pegasus/simulator/params.py"
  grep -q 'PEGASUS_DISABLE_ROS2' "$PEGASUS_EXT/pegasus/simulator/logic/backends/__init__.py"
  echo "[OK] Pegasus patch applied."
elif grep -q 'skip_check=True' "$PEGASUS_EXT/pegasus/simulator/params.py" && \
     grep -q 'PEGASUS_DISABLE_ROS2' "$PEGASUS_EXT/pegasus/simulator/logic/backends/__init__.py"; then
  echo "[SKIP] Pegasus patch already applied."
else
  echo "[ERROR] Patch did not apply; check that Pegasus 5.1.0 is installed." >&2
  exit 1
fi
