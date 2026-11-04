#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image="${COAL_SAM_A_IMAGE:-coal-flyco/sam-a:20261103}"
source_dir="$root/third_party/SAM_AD_Perception"
if ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build -f "$root/docker/Dockerfile.sam-a" -t "$image" "$root/docker/sam-a"
fi
# Downloads only pinned, hash-verified semantic perception dependencies; inference stays offline.
docker run --rm --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$source_dir:/app:rw" -w /app "$image" \
  python -B prepare_assets.py --backend A
docker run --rm --network none --user "$(id -u):$(id -g)" \
  -v "$source_dir:/app:ro" -w /app "$image" \
  python -B -c 'import json; from sam_ad.doctor import verify_manifest; print(json.dumps(verify_manifest(backend="A")))'
