# 真机双机部署：地面电脑与 Orin

依据当前 `dev-real` 分支（审阅提交 `ebdc3a05cfdfd288f1d39a51e9e8e60fea080884`）的实际 launch、配置、构建文件和控制实现整理。

下面采用工程已有的 **Ubuntu 22.04 / ROS2 Humble + Orin + Odin + PX4 原生 FlyCo 模式** 分工。板卡型号、JetPack、固件和接线尚未确认，涉及这些信息的地方给出条件和替换位置；这里的命令未在真机上执行。已有仿真验收见 [dev-real-status.md](dev-real-status.md)，它使用了定制 PX4 固件，未包含 ARM64、双机无线网络和硬件飞行验收。

## 1. 架构与部署边界

```mermaid
flowchart LR
    OD[Odin：设备内 SLAM、RGB、点云] --> SB[Orin：原驱动、sensor_bridge]
    SB --> EV[外部定位：FRD VehicleOdometry]
    EV --> DDS[Orin：MicroXRCEAgent]
    DDS <--> FC[真机 PX4：EKF、位置与姿态控制]
    SB --> SP[Orin：语义感知服务与目标点云]
    SP --> MP[地面电脑：网格预测]
    MP --> PL[Orin：FlyCo 规划与 traj_server]
    PL --> CT[Orin：px4_bridge，动态 FlyCo 模式]
    CT <--> DDS
    PC[地面电脑：UI、RViz、地面心跳] <--> SB
    PC --> CT
    QGC[地面电脑：QGC] <-->|独立 MAVLink 链路| FC
```

| 位置 | 实际入口与职责 |
|---|---|
| `sim/`、`bridge/`、`launch.py` | Isaac/Pegasus 仿真、传感器和 TCP 桥；真机日常启动不经过这一层 |
| `third_party/flyco_ros2` | 沿用的感知、点云聚类、网格预测、覆盖规划及轨迹算法 |
| `ros2/flyco_real` | 真机传感器归一化、TF、PX4 DDS、模式注册、任务调度和按设备拆分的 launch |
| `third_party/drivers/odin_ros_driver` | Odin USB 驱动；按 x86_64 / aarch64 选择 SDK |
| `third_party/px4_msgs` | PX4 消息定义；必须与实际固件协调使用 |
| `third_party/ros_interfaces` | 四个算法消息/服务包；包名里保留 AirSim 不代表真机需要运行 AirSim |
| `third_party/SAM_AD_Perception` | 通过 HTTP 接入的语义模型服务，默认在 Orin 的 `127.0.0.1:8851` |

机载的主数据流为：

```text
/odin1/* → sensor_bridge → /flyco/odom、/flyco/lidar/world、/flyco/camera/*
                       → /fmu/in/vehicle_visual_odometry → PX4 EKF
图像 + 文字 → 本机语义服务 → 目标点云 → 地面预测 → /prediction/predicted_mesh
网格 + 定位 → flyco_planner → traj_server → /planning/pos_cmd
            → px4_bridge → PX4 trajectory_setpoint → 飞控控制器
```

机内使用 ENU / FLU；PX4 接口执行坐标转换和局部原点对齐。外部定位当前只保证位姿，速度和角速度写为 NaN，不应据此开启外部速度融合。

真机运行不需要 Isaac Sim、Pegasus 或 SITL。ROS2 双机链路承载预测输入、网格、可视化和地面心跳；MicroXRCEAgent 承载 Orin 到飞控的数据；QGC 使用独立 MAVLink 链路。启动 Agent 不会自动把飞控连接到 QGC。

## 2. 先收集两端信息

地面电脑执行：

```bash
cat /etc/os-release
uname -m
nvidia-smi
ls /opt/ros
command -v nvcc
```

Orin 执行：

```bash
cat /etc/os-release
uname -m
cat /etc/nv_tegra_release
dpkg-query -W nvidia-jetpack nvidia-l4t-core
python3 --version
ls -l /dev/serial/by-id/
lsusb
```

`nvidia-jetpack` 元包或 `by-id` 目录不存在不等于设备不可用，分别继续检查 L4T 版本和实际 UART 设备。PX4 在 QGC 的 MAVLink Console 中执行：

```text
ver all
uxrce_dds_client status
param show UXRCE_DDS_DOM_ID
param show UXRCE_DDS_CFG
```

当前源码的原生模式依赖 `RegisterExtComponentRequest/Reply`、`ArmingCheckRequest/Reply`、`ModeCompleted`、`vehicle_command_mode_executor` 等接口。不能根据“同为 PX4”或“能看到遥测”判断接口兼容。旧仿真页中的 v1.14.3 是 Pegasus/SITL 说明，不是当前真机动态模式的部署版本要求。

工程已将用户提供的 PX4 1.17 定制源码消息直接放入 `third_party/px4_msgs`，来源为 `feature/neural` 提交 `0d092a91f0c0417ab888b4bc37331484e6d46b9d`，详细校验和见该包 `SOURCE.json`。两端统一 `colcon build`，注册、解锁检查及本地位置接口直接使用对应 `_v1` 话题，不需要额外消息覆盖层或转换节点。实际飞控报告的提交为 `f026a1d…`，仍需机载确认接口运行结果。[PX4 ROS2 消息匹配说明](https://docs.px4.io/v1.16/en/ros2/user_guide)、[原生模式接口](https://docs.px4.io/v1.16/en/ros2/px4_ros2_control_interface)。

## 3. 两端公共准备

以下公共安装块适用于已经安装 ROS2 Humble、已配置 Ubuntu/ROS 软件源的 Ubuntu 22.04。板卡使用其他系统版本时，先处理 ROS / Python ABI；不要把这些 apt 命令当作跨系统安装器。ROS 首次安装见 [Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)。

### 3.1 获取相同源码

地面电脑当前目录就是 `/home/lyl/Coal_FlyCo`，可以直接使用。新设备执行：

```bash
git clone --branch dev-real https://github.com/Yuan-Lin-Liu/Coal_FlyCo.git "$HOME/Coal_FlyCo"
cd "$HOME/Coal_FlyCo"
git rev-parse HEAD
```

两端记录并核对提交；如果远程分支已有更新，应选择同一提交。仓库不包含 `.local/` 内的 Python 环境、权重和 CUDA 扩展，拉取源码不会自动获得这些资源。

### 3.2 安装公共依赖

两端执行：

```bash
sudo apt update
sudo apt install -y \
  build-essential gcc-11 g++-11 cmake ninja-build git curl pkg-config \
  python3-dev python3-pip python3-venv python3-rosdep \
  python3-colcon-common-extensions python3-colcon-defaults \
  python3-numpy python3-scipy python3-opencv python3-yaml python3-transforms3d \
  libeigen3-dev libpcl-dev libompl-dev libyaml-cpp-dev libtbb-dev \
  libopencv-dev libusb-1.0-0-dev libssl-dev libopenblas-dev \
  ros-humble-ros-base ros-humble-rviz2 ros-humble-rmw-fastrtps-cpp \
  ros-humble-cv-bridge ros-humble-geographic-msgs \
  ros-humble-image-transport ros-humble-image-geometry \
  ros-humble-message-filters ros-humble-pcl-conversions \
  ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs \
  ros-humble-sensor-msgs-py ros-humble-rosgraph-msgs chrony
```

安装 `colcon-defaults` 很关键：根目录 `colcon_defaults.yaml` 才会自动限定源码搜索路径，避免递归扫描整个第三方模型树。

Jetson 上还需核对 Odin 与 `cv_bridge` 链接的是同一 OpenCV ABI；NVIDIA 预装版与 ROS apt 版混用可能在首帧图像到达时崩溃，驱动 README 的 5.10 节有排查说明。不要在不知道现有软件依赖的情况下直接替换系统 OpenCV。

### 3.3 创建车辆配置

两端在工程根目录执行；已有配置时保留原文件：

```bash
cp -n ros2/flyco_real/config/odin.yaml config/vehicle.odin.local.yaml
```

先在 Orin 核对配置，再将相同车辆配置同步到地面电脑。需要实测或选择的字段：

| 字段 | 作用 |
|---|---|
| `sensor_bridge.ros__parameters.odometry_is_body_frame` | Odin 配置自动启用；IMU 与机体原点、轴向重合，不做安装外参补偿 |
| `sensor_bridge.ros__parameters.world_from_lio_*` | SLAM 世界系与规划 `world` 的关系 |
| `sensor_bridge.ros__parameters.publish_external_vision` | 是否由本工程向 PX4 发送外部定位，避免和原定位桥重复发布 |
| `px4_bridge.ros__parameters.bounds_min/max` | 基于实际 `world` 坐标的执行边界 |
| `px4_bridge.ros__parameters.max_speed/max_acceleration` | 指令检查上限，需要与规划参数协调 |
| `px4_bridge.ros__parameters.takeoff_height` | 自动起飞高度；模板未写出，代码默认 2.0 m |
| `flyco_planner.ros__parameters.flyco_planner.initial_height` | 规划初始化高度，修改起飞高度时一并核对 |
| `sam_tracking.ros__parameters.segmentation_endpoint` | 默认保持 `http://127.0.0.1:8851`，语义服务在 Orin |

Odin 内部的相机内参、去畸变和传感器间 TF 由原驱动处理。本工程不再启动额外外参适配节点，默认零安装偏移；保留 `world → odom`、`world → base_link` 和驱动原生 TF。桥接图像和 CameraInfo 的帧名统一为原驱动光学 TF `camera_0`，PX4 坐标约定转换继续保留。旧车辆配置中的安装外参字段被忽略。

## 4. Orin 首次部署

### 4.1 构建 ARM64 Embree 3

工程的 ARM 构建要求 Embree **3**。Ubuntu 22.04 的 `libembree-dev` 官方包仅提供 amd64，不能把该包或仓库内的 x86 `.so` 复制到 Orin；Ubuntu 24.04 提供的 Embree 4 也不满足本工程的 `find_package(embree 3 ...)`。[Ubuntu 包信息](https://packages.ubuntu.com/libembree-dev)。

以下是按上游 3.13.5 的 ARM/NEON CMake 选项整理的目标机编译命令，尚待 Orin 构建验收：

```bash
cd "$HOME/Coal_FlyCo"
mkdir -p .local/deploy/src
git clone --branch v3.13.5 --depth 1 \
  https://github.com/RenderKit/embree.git .local/deploy/src/embree
cmake -S .local/deploy/src/embree -B .local/deploy/build/embree -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/.local/deploy/embree3" \
  -DEMBREE_ISPC_SUPPORT=OFF -DEMBREE_TUTORIALS=OFF \
  -DEMBREE_MAX_ISA=NEON
cmake --build .local/deploy/build/embree --parallel 2
cmake --install .local/deploy/build/embree
export CMAKE_PREFIX_PATH="$PWD/.local/deploy/embree3${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$PWD/.local/deploy/embree3/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

已经有本机 ARM64 Embree 3 时复用其安装前缀，无需重新构建。[上游版本与 ARM 检测](https://github.com/RenderKit/embree/blob/v3.13.5/CMakeLists.txt)。

### 4.2 编译 ROS 工作区

在前一步同一终端执行：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 pkg executables flyco_real
ldd install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample
```

默认构建 8 个包，顺序编译、最多两个编译任务。Odin 选择 `liblydHostApi_arm.a`，LKH/kdline 在目标 CPU 编译，旧仿真控制器默认关闭。

### 4.3 配置设备权限

```bash
cd "$HOME/Coal_FlyCo"
sudo usermod -aG dialout,plugdev "$USER"
sudo install -m 0644 ros2/flyco_real/config/udev/99-coal-odin.rules \
  /etc/udev/rules.d/99-coal-odin.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新登录并重新插拔设备后再启动驱动。Odin 使用 USB 权限，飞控/UART 使用串口权限；不要依赖每次启动都变化的 `/dev/ttyACM0` 编号，USB 转串口优先使用 `/dev/serial/by-id/...`，板载 UART 使用载板实际设备名。

### 4.4 安装 DDS Agent

已有与固件匹配且正常工作的 Agent 时复用。以下安装版本来自 PX4 v1.16 官方部署示例；本次定制 PX4 1.17 已连接现有 UDP Agent，继续复用现有安装：

```bash
cd "$HOME/Coal_FlyCo"
git clone --branch v2.4.3 --depth 1 \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git .local/deploy/src/Micro-XRCE-DDS-Agent
cmake -S .local/deploy/src/Micro-XRCE-DDS-Agent \
  -B .local/deploy/build/xrce-agent -GNinja -DCMAKE_BUILD_TYPE=Release
cmake --build .local/deploy/build/xrce-agent --parallel 2
sudo cmake --install .local/deploy/build/xrce-agent
sudo ldconfig
command -v MicroXRCEAgent
```

固件版本不同先核对客户端/Agent 兼容关系，不直接安装最新 Agent。[PX4 Agent 安装说明](https://docs.px4.io/v1.16/en/ros2/user_guide#setup-micro-xrce-dds-agent-client)。

### 4.5 准备语义模型服务

这是当前仓库尚未完成 Orin 验收的部分。`setup_linux.sh` 固定 Python 3.12、Torch 2.7.1、torchvision 0.22.1/cu128；桌面镜像也采用该组合。不要直接在 Jetson 上运行它并期待获得可用的 GPU 环境。模型服务与 ROS Python 3.10 通过 HTTP 隔离，可以使用另一个 Python 环境，但 Torch、torchvision、CUDA 和 Python ABI 必须同时匹配。

先按板卡实际 JetPack，使用 [NVIDIA Jetson PyTorch 安装与兼容说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/) 选择环境。这里不指定未知的 wheel URL。当前模型依赖以 Python 3.12 为基线；若 JetPack 对应的现成 wheel 只有 cp310，不能将其安装进 Python 3.12，需另做受支持环境或源码构建适配。这是完整机载感知部署的前置条件。

已准备好兼容的语义环境后执行，替换第一行路径：

```bash
cd "$HOME/Coal_FlyCo"
export SEMANTIC_PYTHON=/absolute/path/to/jetson-semantic-env/bin/python
"$SEMANTIC_PYTHON" -c 'import sys, torch, torchvision; print(sys.version, torch.__version__, torchvision.__version__, torch.version.cuda); assert sys.version_info >= (3, 12); assert torch.cuda.is_available()'
"$SEMANTIC_PYTHON" -m pip install -r third_party/SAM_AD_Perception/requirements-server.txt
cd third_party/SAM_AD_Perception
"$SEMANTIC_PYTHON" -B prepare_assets.py --backend A
"$SEMANTIC_PYTHON" -B -m sam_ad.doctor --backend A
```

`doctor` 同时检查 CUDA tensor、torchvision GPU NMS 和模型资产完整性。仍需在第 8 节发送真实目标文字，验证端到端推理与时延。权重及锁定的上游源码可以从远程已验证机器按原路径复制，Python 虚拟环境及二进制扩展需要针对本机建立。

## 5. 地面电脑首次部署

### 5.1 编译 ROS 工作区

在已安装第 3 节公共依赖的终端执行：

```bash
cd "$HOME/Coal_FlyCo"
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

地面电脑也默认构建全部包，实际只启动地面角色；x86 默认复用仓库的 x86 Embree 3。

### 5.2 安装地面预测环境

`colcon build` 只构建 ROS 包，不会创建预测 Python 环境或下载模型。新 x86_64 地面电脑可在已完成 ROS 构建后执行：

```bash
bash tools/setup_ground_predictor.sh
# 之后只检查环境和资源，不重新下载：
bash tools/setup_ground_predictor.sh --check
```

脚本将环境安装到 `.local/deploy/predictor`，只下载地面预测需要的模型；缺少 CUDA 11.8 时在 `.local/deploy/cuda-root/usr/local/cuda-11.8` 解包所需编译工具和 cuSPARSE，不安装或替换系统驱动，不设置 ROS 域。首次安装需要联网、GPU 可用以及编译工具。在没有系统 CUDA 11.8、采用此项目内安装时，后面的地面 launch 使用 `cuda_home:="$PWD/.local/deploy/cuda-root/usr/local/cuda-11.8"`。

下面保留分步安装方式，便于已有环境的机器按需复用。

已有能运行本工程的 Python 3.10 / Torch 2.3.1 / CUDA 11.8 / NKSR 环境时可直接复用路径；只需验证下面的导入。全新 x86_64 电脑先保证 NVIDIA 驱动工作且有 CUDA 11.8 的 `nvcc`。CUDA 11.8 安装说明见 [NVIDIA 文档](https://docs.nvidia.com/cuda/archive/11.8.0/cuda-installation-guide-linux/index.html)。

仅当本机缺少该 Toolkit，且确认为 Ubuntu 22.04 x86_64 时，可安装：

```bash
curl -fL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
  -o /tmp/coal-flyco-cuda-keyring.deb
sudo dpkg -i /tmp/coal-flyco-cuda-keyring.deb
sudo apt update
sudo apt install -y cuda-toolkit-11-8
```

RTX 3090/4090 的环境可参考本工程已有验证组合；如果是更新且 CUDA 11.8 不支持的 GPU，需重新适配 Torch/NKSR/扩展，不能继续照用该组合。

以下从 `docker/Dockerfile.flyco` 提取原生预测环境安装步骤：

```bash
cd "$HOME/Coal_FlyCo"
mkdir -p .local/deploy/src
python3 -m venv --system-site-packages .local/deploy/predictor
export PREDICTOR_PYTHON="$PWD/.local/deploy/predictor/bin/python"
export CUDA_HOME=/usr/local/cuda-11.8
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CC=gcc-11 CXX=g++-11 MAX_JOBS=2
"$PREDICTOR_PYTHON" -m pip install 'pip<25' setuptools==69.5.1 wheel
"$PREDICTOR_PYTHON" -m pip install torch==2.3.1 torchvision==0.18.1 \
  --index-url https://download.pytorch.org/whl/cu118
"$PREDICTOR_PYTHON" -m pip install -r docker/requirements.txt
"$PREDICTOR_PYTHON" -m pip install --no-index --no-deps \
  -f https://data.pyg.org/whl/torch-2.3.0+cu118.html torch-scatter==2.1.2+pt23cu118
"$PREDICTOR_PYTHON" -c 'import torch; assert torch.cuda.is_available(); print(torch.cuda.get_device_name(), torch.cuda.get_device_capability())'
```

NKSR 使用与仓库容器相同的固定源码版本：

```bash
git init .local/deploy/src/NKSR
git -C .local/deploy/src/NKSR remote add origin https://github.com/nv-tlabs/NKSR.git
git -C .local/deploy/src/NKSR fetch --depth 1 origin e40336845e67761343a756788e5a98b827d4a143
git -C .local/deploy/src/NKSR checkout FETCH_HEAD
git init .local/deploy/src/NKSR/package/external/openvdb
git -C .local/deploy/src/NKSR/package/external/openvdb remote add origin \
  https://github.com/AcademySoftwareFoundation/openvdb.git
git -C .local/deploy/src/NKSR/package/external/openvdb fetch --depth 1 origin \
  7edd8cd86f105a01a41ad7b2bf59a81034cd79fb
git -C .local/deploy/src/NKSR/package/external/openvdb checkout FETCH_HEAD
git clone --depth 1 --branch 3.4 https://gitlab.com/libeigen/eigen.git \
  .local/deploy/src/NKSR/package/external/eigen
"$PREDICTOR_PYTHON" -m pip install --no-build-isolation .local/deploy/src/NKSR/package
```

准备权重并首次编译 PointNet2 扩展：

```bash
"$PREDICTOR_PYTHON" tools/download_models.py --directory "$PWD/.local/flyco/models"
"$PREDICTOR_PYTHON" tools/download_models.py --directory "$PWD/.local/flyco/models" --huggingface
"$PREDICTOR_PYTHON" tools/download_models.py --directory "$PWD/.local/flyco/models" --check
source /opt/ros/humble/setup.bash
source install/setup.bash
export TORCH_EXTENSIONS_DIR="$PWD/.local/deploy/torch_extensions"
mkdir -p "$TORCH_EXTENSIONS_DIR"
"$PREDICTOR_PYTHON" -c 'import rclpy, cv_bridge, torch, nksr; from pointnet2_ops import pointnet2_utils; print("Ground predictor imports OK")'
```

PointNet2 会在缺少本机兼容二进制时 JIT 编译。首次编译需有 CUDA Toolkit；不要把别台机器的 Torch 扩展缓存直接拿来用。上面的模型工具校验完整仿真模型清单，可能下载比真机地面预测所需更多的资源；已有远程缓存可以复制模型目录后运行 `--check`。

### 5.3 QGC

复用远程验证过、支持动态模式列表的 QGC 版本；v1.16 官方接口文档使用 QGC Daily。启动实际下载的 AppImage：

```bash
chmod +x /absolute/path/to/QGroundControl.AppImage
/absolute/path/to/QGroundControl.AppImage
```

QGC 通过数传、独立串口或已配置的 MAVLink UDP 连接真机。独立链路应在启动 FlyCo 前就能显示车辆。飞控接 Orin 的 USB/TELEM 口若用于 XRCE，不能同时再让 MAVLink 占用同一串口。

## 6. 双机网络、时间与每个终端环境

以下采用 DDS 域 **0** 作为新部署示例；已有通畅网络时沿用原域，并统一修改两端环境与飞控 `UXRCE_DDS_DOM_ID`。`config/odin.yaml` 的 `deployment.ros_domain_id` 是配置记录，当前 launch 不会据此自动设置终端环境。

每个运行终端都先执行：

```bash
cd "$HOME/Coal_FlyCo"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=66
```

当前实机 PX4 的 DDS 域为 66，两端节点均使用该域。本工程已包含配套消息定义，运行终端仅加载本工程环境。

Orin 再加载本机 Embree 前缀；地面再设置预测扩展缓存：

```bash
# 仅 Orin；按第 4.1 节安装时使用
export LD_LIBRARY_PATH="$PWD/.local/deploy/embree3/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

```bash
# 仅地面电脑；与首次编译 PointNet2 使用同一个缓存
export TORCH_EXTENSIONS_DIR="$PWD/.local/deploy/torch_extensions"
```

不要 source `tools/flyco_env.sh`：它为旧仿真设置域 64、`ROS_LOCALHOST_ONLY=1`，并清空覆盖层、注入 x86 库路径。真机命令也不带 `simulation:=true` 或 `--sim`。

两端检查：

```bash
chronyc tracking
chronyc sources -v
ros2 multicast receive
```

另一台机器运行：

```bash
ros2 multicast send
```

两端时钟必须实际同步；无互联网时可让地面电脑作为内网 NTP 源，Orin 指向该源。只检查时区相同不足以保证时间一致。ROS multicast 测试通过后，还需用第 8 节的 ROS 话题验证 DDS 数据传输；无线 AP 客户端隔离、防火墙或 VPN 可能影响发现和大消息传输。

该工程默认把原始 RGB、点云和网格送到地面可视化/预测，需实测无线带宽与时延。不能只凭 `ping` 通判断足够承载完整算法。

## 7. PX4 配置核对

飞控端和 Linux 端是两个配置面：在 Orin 启动 Agent，不会自动启动/配置飞控的 XRCE 客户端。

如果实际接线选择 **PX4 TELEM2 → Orin 串口，921600 baud**，在 QGC 参数页将 `UXRCE_DDS_CFG` 选择为 TELEM2，`SER_TEL2_BAUD` 设为 921600，`UXRCE_DDS_DOM_ID` 设为选定域。停止占用 TELEM2 的其他串口功能；QGC 使用另一条 MAVLink 链路。USB/网口/其他 UART 的配置按实际接口选择，不照搬 TELEM2 参数。

QGC MAVLink Console 先只读检查：

```text
uxrce_dds_client status
param show UXRCE_DDS_CFG
param show UXRCE_DDS_DOM_ID
param show UXRCE_DDS_PTCFG
param show UXRCE_DDS_SYNCT
param show EKF2_EV_CTRL
param show EKF2_HGT_REF
param show EKF2_EV_DELAY
param show EKF2_EV_POS_X
param show EKF2_EV_POS_Y
param show EKF2_EV_POS_Z
```

`UXRCE_DDS_PTCFG=1` 会把 DDS 限制到 localhost，需要与双机通信方式协调；外部定位时间戳依赖 XRCE 时间同步，需核对 `UXRCE_DDS_SYNCT`。外部定位融合选择水平位置、需要时的高度与航向；当前输出不支持外部速度融合。延迟和安装补偿依据实际链路设置，收到 `/fmu/in/vehicle_visual_odometry` 不代表 EKF 已融合成功。变更需重启的参数后重启并重新检查。

本机的 `sensor_bridge` 已将定位变换至 `base_link`；核对 PX4 的定位偏移参数，避免再次补偿 Odin 的同一杆臂。依据：[XRCE 配置](https://docs.px4.io/v1.16/en/middleware/uxrce_dds)、[外部定位融合](https://docs.px4.io/v1.16/en/ros/external_position_estimation)。

## 8. 日常启动命令：按顺序、每块一个终端

每个终端先执行第 6 节公共环境。下面假设第 4/5 节已完成，并已核对固件接口、车辆安装关系和 PX4 外部定位融合。

### 8.1 Orin A：DDS Agent

串口连接时替换设备名，波特率与飞控端一致：

```bash
ros2 launch flyco_real agent.launch.py \
  transport:=serial device:=/dev/serial/by-id/REPLACE_WITH_ACTUAL_DEVICE baudrate:=921600
```

如果实际是 UDP 连接，则改用下方命令，并在 PX4 中配置 Agent 为 Orin 的真实 IP；二者只运行实际使用的一个：

```bash
ros2 launch flyco_real agent.launch.py transport:=udp4 port:=8888
```

已有 Agent 时复用，不重复启动同一通道的 Agent。

### 8.2 Orin B：Odin 定位、标定与传感器桥

```bash
ros2 launch flyco_real odin.launch.py config:="$PWD/config/vehicle.odin.local.yaml"
```

该入口已经包含 sensor_bridge 和相机视锥。不要再启动会重复创建这些节点的组合入口。先确认 `/flyco/odom` 以及 PX4 的本地位置估计正常。

### 8.3 Orin C：语义推理服务

使用第 4.5 节准备好的实际解释器：

```bash
ros2 launch flyco_real semantic.launch.py \
  python:=/absolute/path/to/jetson-semantic-env/bin/python
```

另一个 Orin 终端检查就绪：

```bash
ros2 run flyco_real check_semantic.py \
  --config "$PWD/config/vehicle.odin.local.yaml" --wait --timeout 180
```

如设置了 `SAM_AD_API_TOKEN`，服务和机载算法终端需使用相同 token。

### 8.4 地面 A：预测、UI、RViz 与心跳

```bash
ros2 launch flyco_real ground.launch.py \
  project_root:="$PWD" \
  config:="$PWD/config/vehicle.odin.local.yaml" \
  model_python:="$PWD/.local/deploy/predictor/bin/python" \
  models_dir:="$PWD/.local/flyco/models" \
  cuda_home:="$PWD/.local/deploy/cuda-root/usr/local/cuda-11.8" \
  cache_dir:="$PWD/.local/deploy/torch_extensions" \
  start_semantic:=false ui:=true rviz:=true
```

上述 CUDA 路径对应第 5 节的一键安装脚本；如使用自行安装的完整 CUDA，则将 `cuda_home` 改为实际目录。也可以省略该参数，由启动文件在配置路径不可用时查找项目内 CUDA。不要传入不存在的 `/usr/local/cuda-11.8`。

此终端会发布 `/flyco/ground/heartbeat`，运行期间保持开启。另起 QGC 并接入真机。界面默认不在地面启动语义服务，机载配置的 HTTP 地址仍指向 Orin 自身。

### 8.5 Orin D：模式注册，首次先保持手动使能

首次地面联调建议先关闭自动使能，以便观察模式注册和定位；此入口不会自动解锁：

```bash
ros2 launch flyco_real mode.launch.py \
  config:="$PWD/config/vehicle.odin.local.yaml" auto_enable:=false
```

定位、网络和模式检查完成后可以在另一终端显式使能：

```bash
ros2 run flyco_real task.py enable
```

完成首轮联调后，停止上面的独立模式节点，日常使用下一节的合并入口即可完成模式注册、自动使能和解锁后启动机载算法。

### 8.6 Orin：合并注册、解锁后启动机载算法与任务节点

```bash
ros2 launch flyco_real flyco.launch.py config:="$PWD/config/vehicle.odin.local.yaml"
```

该入口复用已有定位桥，立即启动模式注册和控制桥。选择 FlyCo 并正常解锁后，自动启动规划、轨迹服务器、目标跟踪/聚类和任务调度；初始化完成输出 `FlyCo ready`。不要同时运行另一份模式注册节点；若继续复用上一节的独立模式节点，给命令增加 `start_mode:=false`。

### 8.7 两端观察链路

在已加载环境的诊断终端按需要分别运行：

```bash
ros2 topic hz /flyco/odom
ros2 topic hz /flyco/ground/heartbeat
ros2 run flyco_real task.py status
ros2 topic echo /flyco/control/status --once
ros2 topic echo /flyco/mission/status --once
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo world camera_0
```

PX4 话题可能带 `_vN`，先查看实际名字；代码会根据消息的 `MESSAGE_VERSION` 自动选择后缀：

```bash
ros2 topic list -t | rg '/fmu/|/flyco/|/prediction/'
```

对 PX4 输出使用 `ros2 topic echo <实际话题名> --qos-reliability best_effort --once`，避免默认 QoS 不匹配导致误判“没有数据”。模式状态优先查看 `registered_mode_id`、`registration_issue`、`px4_ok`、`odom_ok`、`ground_ok`、`enabled` 和 `reason`。地面执行相同诊断应能看到 Orin 的数据。

## 9. 从联调到任务操作

先在不解锁、拆桨的状态完成设备、坐标、PX4 融合、双机预测及模式注册检查；确认手动移动机体时 RViz/PX4 的位置方向和姿态一致。

进入实际飞行阶段后，工程的常规触发方式是：**选择 FlyCo → 地面正常解锁 → 自动启动算法并初始化 → PX4 Takeoff → 约 2 m 后返回 FlyCo 悬停**。也支持先在 Position 解锁再切入 FlyCo。launch 本身不解锁；空中激活、地面状态未知或初始化期间取消解锁/模式时不自动起飞。

悬停后，在地面已加载环境的终端执行：

```bash
ros2 run flyco_real task.py text 'A single large pile of black coal.'
```

任务节点等待新目标点云和 `/prediction/predicted_mesh` 就绪后，自动开启规划、扫描和轨迹跟踪；规划器报告扫描完成后保持当前位置。

需要时分别执行：

```bash
ros2 run flyco_real task.py hold
ros2 run flyco_real task.py resume
ros2 run flyco_real task.py land
ros2 run flyco_real task.py status
```

降落请求返回成功仍需观察实际落地、上锁和停桨，之后再结束算法、模式、感知和定位节点。地面心跳默认超时为 5 秒；执行扫描时链路中断会停止跟踪并保持，模式健康状态也会受到影响。真机的最终失效动作由 PX4 配置决定，不能把仿真里额外请求就地 Land 的策略当作真机已有行为。

## 10. 目前仍需针对硬件落实的三项

1. Orin 型号/内存、JetPack 和可运行当前语义模型的 Python/Torch/torchvision 组合；当前没有经过验证的 Orin 模型安装器。
2. 真机 PX4 的准确版本、原生模式 DDS 接口与匹配的 `px4_msgs`；仓库仿真验收使用的定制固件不能自动代表真机固件。
3. 飞控接线与参数、双机网卡和时间源；这些信息决定通信、域、融合和运行配置。Odin 当前按零安装偏移使用。

上述条件确认后，第 8 节就是日常双机启动顺序；无需再启动仿真管理器。
