#!/usr/bin/env bash
# Build environments from recipes; weights and caches remain outside Git.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"
for tool in docker python3 curl nvidia-smi; do
  command -v "$tool" >/dev/null || { echo "Missing host prerequisite: $tool" >&2; exit 1; }
done
[[ "$(uname -m)" == x86_64 ]] || { echo 'This release supports Linux x86_64.' >&2; exit 1; }
python3 -c 'import yaml' || { echo 'Install host python3-yaml first.' >&2; exit 1; }
docker info >/dev/null
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
ISAAC_IMAGE="${COAL_ISAAC_IMAGE:-coal-flyco/isaac:5.1.0-v1}"
FLYCO_IMAGE="${COAL_FLYCO_IMAGE:-coal-flyco/flyco:5.1.0-v1}"
if [[ "${1:-}" == --check ]]; then
  docker image inspect "$ISAAC_IMAGE" "$FLYCO_IMAGE" >/dev/null
  python3 tools/download_models.py --check
  echo 'Local images and model resources are ready.'
  exit 0
fi
[[ $# -eq 0 || "${1:-}" == --rebuild ]] || { echo 'Usage: setup.sh [--check|--rebuild]' >&2; exit 2; }
[[ "${ACCEPT_EULA:-}" == Y ]] || { echo 'Read the NVIDIA Isaac Sim EULA; then run ACCEPT_EULA=Y ./docker/setup.sh.' >&2; exit 1; }
arch="${TORCH_CUDA_ARCH_LIST:-$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | sort -u | tr '\n' ';' | sed 's/;$//')}"
[[ -n "$arch" ]] || { echo 'Cannot detect GPU architecture; set TORCH_CUDA_ARCH_LIST.' >&2; exit 1; }
if [[ "${1:-}" == --rebuild ]] || ! docker image inspect "$ISAAC_IMAGE" >/dev/null 2>&1; then
  docker build --progress=plain -f docker/Dockerfile.isaac -t "$ISAAC_IMAGE" .
fi
if [[ "${1:-}" == --rebuild ]] || ! docker image inspect "$FLYCO_IMAGE" >/dev/null 2>&1; then
  docker build --progress=plain -f docker/Dockerfile.flyco \
    --build-arg "TORCH_CUDA_ARCH_LIST=$arch" --build-arg "BUILD_JOBS=${BUILD_JOBS:-4}" -t "$FLYCO_IMAGE" .
fi
python3 tools/download_models.py
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PROJECT_ROOT/.local/flyco/models:/models" \
  -e HF_HOME=/models/huggingface -e HOME=/tmp \
  "$FLYCO_IMAGE" /opt/flyco-runtime/venv/bin/python \
  /workspace/coal_flyco/tools/download_models.py --huggingface --directory /models
python3 tools/download_models.py --check
echo 'Ready. Run: ACCEPT_EULA=Y ./docker/run.sh --scene 4'
