#!/usr/bin/env bash
# Native semantic environment for Jetson Linux R36.4 / CUDA 12.6 / Python 3.10.
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sam_root="$project_root/third_party/SAM_AD_Perception"
sam_python="${SAM_AD_PYTHON:-/usr/bin/python3.10}"
sam_cuda="${CUDA_HOME:-/usr/local/cuda-12.6}"

if [[ "$(uname -m)" != aarch64 ]] || ! grep -Eq 'R36 .*REVISION: 4\.' /etc/nv_tegra_release 2>/dev/null; then
  echo 'This installer requires Jetson Linux R36.4 (ARM64).' >&2
  exit 1
fi
if [[ ! -x "$sam_cuda/bin/nvcc" ]] || ! "$sam_cuda/bin/nvcc" --version | grep -q 'release 12.6'; then
  echo 'Install CUDA 12.6 first, or set CUDA_HOME to its installation directory.' >&2
  exit 1
fi
"$sam_python" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Python 3.10 is required by the Jetson wheels"'

sudo apt-get install -y --no-install-recommends \
  python3.10-venv python3.10-dev libopenblas-dev libcudnn9-cuda-12
if [[ ! -x "$sam_root/.venv/bin/python" ]]; then
  "$sam_python" -m venv "$sam_root/.venv"
fi
model_python="$sam_root/.venv/bin/python"
"$model_python" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Existing .venv must use Python 3.10; preserve it under another name before installing"'

export CUDA_HOME="$sam_cuda"
export PATH="$sam_cuda/bin:$PATH"
export LD_LIBRARY_PATH="$sam_cuda/lib64:${LD_LIBRARY_PATH:-}"
export PIP_DEFAULT_TIMEOUT="${PIP_DEFAULT_TIMEOUT:-120}"

# Keep the desktop requirements intact; only adapt the server environment.
"$model_python" - "$sam_root" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1])
requirements = (root/'requirements-server.txt').read_text()
requirements = requirements.replace('scikit-image==0.26.0', 'scikit-image==0.25.2')
requirements = requirements.replace('opencv-python==', 'opencv-python-headless==')
(root/'.venv/requirements-jetson.txt').write_text(requirements)
(root/'.venv/constraints-jetson.txt').write_text(
    'torch==2.8.0\ntorchvision==0.23.0\nnumpy==1.26.4\n')
PY

"$model_python" -m pip install --index-url https://pypi.org/simple --upgrade pip setuptools wheel
# Jetson AI Lab JP6/cu126 ARM64 builds. Hashes pin these exact CUDA packages.
"$model_python" -m pip install --index-url https://pypi.org/simple \
  --constraint "$sam_root/.venv/constraints-jetson.txt" \
  'https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl#sha256=62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc' \
  'https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl#sha256=907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1' \
  --requirement "$sam_root/.venv/requirements-jetson.txt"

# Download only backend A; existing verified assets are reused.
"$model_python" -B "$sam_root/prepare_assets.py" --backend A
echo 'Semantic environment installed. Start from the project root:'
echo 'ros2 launch flyco_real semantic.launch.py'
