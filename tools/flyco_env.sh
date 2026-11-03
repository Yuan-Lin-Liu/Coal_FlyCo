#!/usr/bin/env bash
# One project-local FlyCo environment. Only system ROS/toolchains stay external.
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export FLYCO_SOURCE="$PROJECT_ROOT/third_party/flyco_ros2"
export FLYCO_RUNTIME="${FLYCO_RUNTIME:-$PROJECT_ROOT/.local/flyco}"
export FLYCO_MODELS="${FLYCO_MODELS:-$FLYCO_RUNTIME/models}"
export FLYCO_CACHE="${FLYCO_CACHE:-$FLYCO_RUNTIME/cache}"
export FLYCO_PYTHON="$FLYCO_RUNTIME/venv/bin/python"

# Do not inherit another workspace's ROS, Python or library overlays.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH PKG_CONFIG_PATH
unset HF_HUB_CACHE HUGGINGFACE_HUB_CACHE TRANSFORMERS_CACHE
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
if [[ -f "$FLYCO_RUNTIME/interfaces/install/local_setup.bash" ]]; then
  source "$FLYCO_RUNTIME/interfaces/install/local_setup.bash"
fi
export AMENT_PREFIX_PATH="$FLYCO_RUNTIME/install:${AMENT_PREFIX_PATH:-}"
export PATH="$FLYCO_RUNTIME/venv/bin:$PATH"
export VIRTUAL_ENV="$FLYCO_RUNTIME/venv"
PERCEPTION_SOURCE="$FLYCO_SOURCE/vendor/perception_predictor/src"
export PYTHONPATH="$FLYCO_SOURCE/python:$PERCEPTION_SOURCE/sam_tracking/src:$PERCEPTION_SOURCE/mesh_predict/src:${FLYCO_SAM_SOURCE:-$PERCEPTION_SOURCE/sam_tracking/thirdparty/sam2-realtime}:$PERCEPTION_SOURCE/sam_tracking/thirdparty/sam_tk:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$FLYCO_RUNTIME/build:$FLYCO_RUNTIME/deps/root/usr/lib/x86_64-linux-gnu:$FLYCO_RUNTIME/deps/cuda-11.8/lib64:$FLYCO_SOURCE/vendor/planner_simulator/src/3rdparty/embree-3/lib:${LD_LIBRARY_PATH:-}"
export CUDA_HOME="${FLYCO_CUDA_HOME:-$FLYCO_RUNTIME/deps/cuda-11.8}"
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:$LD_LIBRARY_PATH"
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.6}"
export CC=gcc-11 CXX=g++-11 MAX_JOBS="${MAX_JOBS:-2}"
export ROS_DOMAIN_ID="${FLYCO_ISAAC_DOMAIN:-64}" ROS_LOCALHOST_ONLY=1
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS RCUTILS_COLORIZED_OUTPUT=0
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export HF_HOME="$FLYCO_MODELS/huggingface"
export TORCH_HOME="$FLYCO_MODELS/torch"
export XDG_CACHE_HOME="$FLYCO_CACHE"
export ROS_LOG_DIR="${FLYCO_ISAAC_RUNTIME:-$PROJECT_ROOT/runtime}/ros_logs"
export FLYCO_EVF_ROOT="$PROJECT_ROOT/third_party/EVF-SAM"
export MESH_PREDICT_MESH_CKPT="$FLYCO_MODELS/mesh/checkpoint.pt"
export MESH_PREDICT_NKSR_CKPT="$FLYCO_MODELS/mesh/snet-n3k-wonormal.pth"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4
export TORCH_EXTENSIONS_DIR="$FLYCO_CACHE/torch_extensions"
export MPLCONFIGDIR="$FLYCO_CACHE/matplotlib"
mkdir -p "$ROS_LOG_DIR" "$TORCH_EXTENSIONS_DIR" "$MPLCONFIGDIR"
