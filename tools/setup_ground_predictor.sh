#!/usr/bin/env bash
# Project-local x86 ground predictor; preserves the caller's ROS domain/settings.
set -eo pipefail
ground_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ground_root"
ground_deploy="$ground_root/.local/deploy"
ground_python="$ground_deploy/predictor/bin/python"
ground_models="${FLYCO_MODELS:-$ground_root/.local/flyco/models}"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONNOUSERSITE=1
export TORCH_EXTENSIONS_DIR="$ground_deploy/torch_extensions"
export MAX_JOBS="${MAX_JOBS:-2}"
export CC=gcc-11 CXX=g++-11

ground_cuda_headers() {
  local ground_header
  for ground_header in "$ground_deploy"/predictor/lib/python*/site-packages/nvidia/*/include; do
    [[ -d "$ground_header" ]] && export CPATH="$ground_header${CPATH:+:$CPATH}"
  done
}

if [[ "$(uname -m)" != x86_64 ]]; then
  echo 'This installer prepares the x86_64 ground computer, not Jetson.' >&2
  exit 1
fi
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), "ROS Humble predictor needs system Python 3.10"'
if [[ ! -f install/setup.bash ]]; then
  echo 'Build the ROS workspace with colcon build first.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p "$ground_deploy/src" "$ground_deploy/downloads" "$TORCH_EXTENSIONS_DIR"

if [[ "${1:-}" != --check ]]; then
  for command in curl git gcc-11 g++-11 cmake ninja dpkg-deb; do
    command -v "$command" >/dev/null || { echo "Missing build tool: $command" >&2; exit 1; }
  done
  if [[ ! -x "$ground_python" ]]; then
    /usr/bin/python3 -m venv --system-site-packages "$ground_deploy/predictor"
  fi
  if [[ ! -x "${CUDA_HOME:-/usr/local/cuda-11.8}/bin/nvcc" || ! -f "${CUDA_HOME:-/usr/local/cuda-11.8}/include/cusparse.h" ]]; then
    # Extract only the required CUDA packages. No driver or system package changes.
    /usr/bin/python3 - "$ground_deploy" <<'PY'
import gzip
import hashlib
import subprocess
import sys
from pathlib import Path

deploy = Path(sys.argv[1])
target = deploy / 'cuda-root'
if ((target / 'usr/local/cuda-11.8/bin/nvcc').is_file()
        and (target / 'usr/local/cuda-11.8/include/cusparse.h').is_file()):
    sys.exit(0)
base = 'https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/'
index = deploy / 'downloads/cuda-Packages.gz'
subprocess.run(['curl', '-fLsS', '--retry', '3', base + 'Packages.gz', '-o', str(index)], check=True)
wanted = {'cuda-nvcc-11-8', 'cuda-cudart-dev-11-8', 'cuda-cudart-11-8',
          'cuda-driver-dev-11-8', 'cuda-cccl-11-8',
          'libcusparse-11-8', 'libcusparse-dev-11-8'}
packages = {}
for paragraph in gzip.decompress(index.read_bytes()).decode().split('\n\n'):
    entry = dict(line.split(': ', 1) for line in paragraph.splitlines()
                 if ': ' in line and not line.startswith(' '))
    if (entry.get('Package') in wanted
            and entry.get('Version') in ('11.8.89-1', '11.7.5.86-1')):
        packages[entry['Package']] = entry
if packages.keys() != wanted:
    raise RuntimeError('Official index is missing pinned CUDA 11.8 packages')
for name, entry in sorted(packages.items()):
    archive = deploy / 'downloads' / Path(entry['Filename']).name
    expected = entry['SHA256']
    def valid():
        return archive.is_file() and hashlib.sha256(archive.read_bytes()).hexdigest() == expected
    if not valid():
        print('Downloading ' + name, flush=True)
        subprocess.run(['curl', '-fLsS', '--retry', '5', '--continue-at', '-',
                        base + entry['Filename'].removeprefix('./'), '-o', str(archive)], check=True)
    if not valid():
        raise RuntimeError('CUDA package SHA256 mismatch: ' + name)
    subprocess.run(['dpkg-deb', '-x', str(archive), str(target)], check=True)
PY
    export CUDA_HOME="$ground_deploy/cuda-root/usr/local/cuda-11.8"
  else
    export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-11.8}"
  fi
  export PATH="$CUDA_HOME/bin:$PATH"
  export LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  "$ground_python" -m pip install --no-cache-dir --disable-pip-version-check \
    'pip<25' setuptools==69.5.1 wheel
  # Prefer already verified local GPU wheels over an equivalent remote index link.
  ground_cached_gpu=()
  for ground_wheel in "$ground_deploy"/downloads/nvidia*cu11-*.whl "$ground_deploy"/downloads/triton-2.3.1-*.whl; do
    [[ -f "$ground_wheel" ]] && ground_cached_gpu+=("$ground_wheel")
  done
  if [[ ${#ground_cached_gpu[@]} -gt 0 ]]; then
    "$ground_python" -m pip install --no-cache-dir --no-index --no-deps "${ground_cached_gpu[@]}"
  fi
  mapfile -t ground_cached_packages < <(/usr/bin/python3 - "$ground_deploy/downloads" <<'PY'
from pathlib import Path
import sys
wanted = {}
for line in Path('tools/requirements-ground.txt').read_text().splitlines():
    if line and not line.startswith('#'):
        name, separator, version = line.partition('==')
        wanted[name.lower().replace('_', '-')] = version if separator else None
for wheel in sorted(Path(sys.argv[1]).glob('*.whl')):
    name, version, _ = wheel.name.split('-', 2)
    name = name.lower().replace('_', '-')
    if name in wanted and (wanted[name] is None or version.split('+')[0] == wanted[name]):
        print(wheel)
PY
  )
  if [[ ${#ground_cached_packages[@]} -gt 0 ]]; then
    "$ground_python" -m pip install --no-cache-dir --no-index --no-deps "${ground_cached_packages[@]}"
  fi
  if ! "$ground_python" -c 'import torch, torchvision; assert torch.__version__.split("+")[0] == "2.3.1"; assert torchvision.__version__.split("+")[0] == "0.18.1"' 2>/dev/null; then
    "$ground_python" -m pip install --no-cache-dir --no-deps --disable-pip-version-check \
      --find-links "$ground_deploy/downloads" \
      torch==2.3.1 torchvision==0.18.1 --index-url https://download.pytorch.org/whl/cu118
  fi
  "$ground_python" -m pip install --no-cache-dir --disable-pip-version-check \
    --find-links "$ground_deploy/downloads" -r tools/requirements-ground.txt
  export TORCH_CUDA_ARCH_LIST="$("$ground_python" -c 'import torch; assert torch.cuda.is_available(), "CUDA GPU unavailable"; print("%d.%d" % torch.cuda.get_device_capability())')"
  "$ground_python" -m pip install --no-cache-dir --no-index --no-deps \
    -f https://data.pyg.org/whl/torch-2.3.0+cu118.html torch-scatter==2.1.2+pt23cu118
  ground_cuda_headers
  if ! "$ground_python" -c 'import nksr' 2>/dev/null; then
    ground_nksr="$ground_deploy/src/NKSR"
    if [[ ! -d "$ground_nksr/.git" ]]; then
      git init "$ground_nksr"
      git -C "$ground_nksr" remote add origin https://github.com/nv-tlabs/NKSR.git
    fi
    git -C "$ground_nksr" fetch --depth 1 origin e40336845e67761343a756788e5a98b827d4a143
    git -C "$ground_nksr" checkout FETCH_HEAD
    if [[ ! -d "$ground_nksr/package/external/openvdb/.git" ]]; then
      git init "$ground_nksr/package/external/openvdb"
      git -C "$ground_nksr/package/external/openvdb" remote add origin https://github.com/AcademySoftwareFoundation/openvdb.git
    fi
    git -C "$ground_nksr/package/external/openvdb" fetch --depth 1 origin 7edd8cd86f105a01a41ad7b2bf59a81034cd79fb
    git -C "$ground_nksr/package/external/openvdb" checkout FETCH_HEAD
    if [[ ! -d "$ground_nksr/package/external/eigen/.git" ]]; then
      git clone --depth 1 --branch 3.4 https://gitlab.com/libeigen/eigen.git "$ground_nksr/package/external/eigen"
    fi
    "$ground_python" -m pip install --no-cache-dir --no-build-isolation --no-deps "$ground_nksr/package"
  fi
  "$ground_python" tools/download_models.py --profile ground --directory "$ground_models"
  "$ground_python" tools/download_models.py --profile ground --directory "$ground_models" --huggingface
elif [[ "${1:-}" == --check ]]; then
  export CUDA_HOME="${CUDA_HOME:-$ground_deploy/cuda-root/usr/local/cuda-11.8}"
  if [[ ! -x "$CUDA_HOME/bin/nvcc" && -x /usr/local/cuda-11.8/bin/nvcc ]]; then
    export CUDA_HOME=/usr/local/cuda-11.8
  fi
  export PATH="$CUDA_HOME/bin:$PATH"
  export LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  ground_cuda_headers
fi
if [[ -z "${POINTNET2_OPS_EXTENSION_PATH:-}" && -f "$TORCH_EXTENSIONS_DIR/_ext/_ext.so" ]]; then
  export POINTNET2_OPS_EXTENSION_PATH="$TORCH_EXTENSIONS_DIR/_ext/_ext.so"
fi
"$ground_python" - <<'PY'
import rclpy, cv_bridge, torch, torchvision, nksr
assert torch.cuda.is_available(), 'CUDA GPU unavailable'
from pointnet2_ops import pointnet2_utils
from mesh_predict.ros.node import MeshPredictNode
boxes = torch.tensor([[0., 0., 2., 2.]], device='cuda')
assert torchvision.ops.nms(boxes, torch.ones(1, device='cuda'), .5).numel() == 1
points = torch.rand((1, 32, 3), device='cuda')
assert tuple(pointnet2_utils.furthest_point_sample(points, 8).shape) == (1, 8)
torch.cuda.synchronize()
print('Predictor imports OK:', torch.cuda.get_device_name(), torch.__version__, torchvision.__version__)
PY
"$ground_python" tools/download_models.py --profile ground --directory "$ground_models" --check
printf 'Ground predictor ready. Python: %s\nCUDA: %s\nModels: %s\n' "$ground_python" "$CUDA_HOME" "$ground_models"
