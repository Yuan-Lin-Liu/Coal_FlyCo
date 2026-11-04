# Orin + PX4 v1.16 原生部署说明（不使用 Docker）

适用目标：Orin 运行驱动、定位、机载感知与规划；地面 RTX 电脑运行预测、可视化。两端都可以原生安装。本文依据当前仓库源码编写，**尚未在 Orin 和实际飞控上完成部署验收**。

目前能直接做的是环境安装、拉取源码、自定义消息编译和桌面语义服务启动。完整 Orin 编译、真机启动和跨机飞行还需完成第 5、7 节的适配。下文明确标出了这些边界。

## 1. PX4、px4ctrl 与当前仿真是什么关系

| 名称 | 运行位置 | 职责 |
|---|---|---|
| PX4 v1.16 固件 | Pixhawk 等实际飞控板 | 状态估计、姿态/角速度控制、执行器输出、解锁、飞行模式和失联保护 |
| `px4ctrl_node`（px4_ctrl） | Orin / 当前仿真电脑 | FlyCo 的外部轨迹跟踪控制器；不是 PX4 固件，也不能烧录到飞控板 |
| MAVROS | Orin | 通过 MAVLink 连接 ROS2 和 PX4；不是飞控 |
| Pegasus + `sim/flight_backend.py` | 当前 Isaac 仿真电脑 | 当前版本的无人机物理与姿态执行适配；没有运行 PX4 固件/SITL |

真机需要飞控板上已经有 PX4 固件。你若已装好适配该飞控型号的 v1.16，一般只需配置，不必为了运行 FlyCo 重新编译烧录。尚未安装时，可用 QGroundControl 加载该飞控对应的 v1.16 固件。只有修改 PX4 自身或支持定制硬件时，才需要单独克隆、编译 PX4-Autopilot；它不属于本 ROS 工作空间。[固件安装](https://docs.px4.io/v1.16/en/config/firmware)、[固件编译](https://docs.px4.io/v1.16/en/dev_setup/building_px4)。

**当前控制输出不能直接接真实 PX4。** `controller.cpp` 将 `roll`、`pitch` 角度写入 `body_rate.x/y`，由 Isaac 适配器按角度解释；而 MAVROS/PX4 的 `AttitudeTarget.body_rate` 表示 rad/s。还存在未启用的 RC/模式/超时状态机。前一版指南仅强调 remap，遗漏了这个关键差异；现已纠正。详情见第 7 节。

论文的定位引用是 **FAST-LIO2**；硬件包含 Mid360、IMU 和云台 RGB 相机，预测放在地面电脑。论文原平台是改装 DJI M30；PX4 v1.16 是本项目拟适配的真机方案。当前仓库没有 FAST-LIO2/传感器驱动，也未接入 Super-LIO。[论文 §5.1.3、§7.1.1](https://arxiv.org/html/2601.07558v1)。

```mermaid
flowchart LR
    S[雷达 IMU 相机] --> O[Orin 定位与感知]
    O --> P[FlyCo 规划 / traj_server]
    O --> G[地面预测]
    G --> P
    P --> A[真机控制适配]
    A --> M[MAVROS]
    M --> F[飞控板 PX4 v1.16]
    F --> E[电调 / 电机]
```

## 2. 目录布局：驱动放在工作空间的哪里

建议使用同一 `main` 和相同提交号，用工作空间/启动配置区分角色：

```text
~/uav/
├── repos/coal_flyco/               # 本仓库，保留 Git 管理
├── third_party/Livox-SDK2/         # 厂商 C/C++ SDK，独立 CMake 构建
├── drivers_ws/src/
│   ├── livox_ros_driver2/          # ROS2 雷达驱动
│   ├── camera_driver/             # 按实际相机选择，示意目录
│   └── gimbal_driver/             # 按实际云台选择，示意目录
├── localization_ws/src/
│   └── fast_lio_ros2/             # 选定、验证并固定版本的 FAST-LIO2 ROS2 移植
├── flight_ws/src/
│   ├── ros_interfaces -> ../../repos/coal_flyco/third_party/ros_interfaces
│   ├── flyco_ros2 -> ../../repos/coal_flyco/third_party/flyco_ros2
│   └── flyco_real_bringup/         # 待开发：实机配置、坐标适配、PX4 接口、launch
├── models/                        # 模型权重，不放 src，不复制电脑的 venv
└── firmware/PX4-Autopilot/         # 仅需要自行编译 PX4 时使用，可放地面电脑
```

`build/`、`install/`、`log/` 位于各工作空间根目录，与 `src/` 并列；不要放进源码仓库。ROS 驱动放 `drivers_ws/src/`；非 ROS 的 SDK 放独立目录并安装到系统/专用前缀。MAVROS 用 apt 安装时在 `/opt/ros/humble`，不必再复制源码。

每个新终端按“系统 → 驱动 → 定位 → 业务”顺序加载已经存在的环境：

```bash
source /opt/ros/humble/setup.bash
source ~/uav/drivers_ws/install/setup.bash
source ~/uav/localization_ws/install/setup.bash
source ~/uav/flight_ws/install/setup.bash
export ROS_DOMAIN_ID=64
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

安装初期只 source 已完成编译的工作空间。不要在此后 source 当前 `tools/flyco_env.sh`：它会清除其他 overlay，并把通信限制为本机；应为实机新增独立环境脚本。

## 3. 需要安装哪些环境

| 设备 | 基础环境 | Python / GPU |
|---|---|---|
| Orin | JetPack 6.x、其配套 Ubuntu 22.04 arm64、ROS2 Humble、MAVROS | ROS 节点用系统 Python 3.10；按需安装与 JetPack 匹配的 Jetson PyTorch，Orin CUDA 架构为 8.7 |
| 地面电脑 | Ubuntu 22.04 x86_64、ROS2 Humble、兼容的 NVIDIA 驱动 | 预测用 Python 3.10 + Torch 2.3.1/cu118；当前语义服务另用 Python 3.12 + Torch 2.7.1/cu128 |
| 飞控板 | 与板卡型号匹配的 PX4 v1.16 | 不安装 ROS、Python、CUDA |

先按 [ROS2 Humble 官方安装步骤](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)配置 ROS apt 源，再在两端安装：

```bash
sudo apt update
sudo apt install -y git curl build-essential cmake ninja-build pkg-config \
  python3-dev python3-venv python3-pip python3-yaml python3-opencv \
  python3-scipy python3-transforms3d python3-colcon-common-extensions \
  python3-rosdep libeigen3-dev libpcl-dev libopencv-dev libompl-dev \
  libyaml-cpp-dev libtbb-dev \
  ros-humble-ros-base ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-cv-bridge ros-humble-image-geometry ros-humble-image-transport \
  ros-humble-pcl-conversions ros-humble-message-filters \
  ros-humble-geographic-msgs ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-sensor-msgs ros-humble-sensor-msgs-py
source /opt/ros/humble/setup.bash
sudo ros2 run mavros install_geographiclib_datasets.sh
```

地面可视化另装 `ros-humble-rviz2`。首次使用 rosdep 时执行 `sudo rosdep init`、`rosdep update`；已初始化的不重复执行 init。Orin 的 CUDA/驱动由 JetPack 提供，不能照搬桌面机驱动安装包。[Jetson PyTorch 版本表](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)、[MAVROS 安装](https://github.com/mavlink/mavros/blob/ros2/README.md)。

## 4. 拉取代码，只编译需要的部分

两端都先记录同一个发布提交号：

```bash
mkdir -p ~/uav/repos ~/uav/flight_ws/src
cd ~/uav/repos
git clone https://github.com/Yuan-Lin-Liu/Coal_FlyCo.git coal_flyco
cd coal_flyco
git rev-parse HEAD
```

不需要删除煤仓资源来编译。把下列目录链接到 `flight_ws/src/`，colcon 就只发现选定的包，不启动 Isaac：

```bash
ln -s ~/uav/repos/coal_flyco/third_party/ros_interfaces ~/uav/flight_ws/src/ros_interfaces
ln -s ~/uav/repos/coal_flyco/third_party/flyco_ros2 ~/uav/flight_ws/src/flyco_ros2
```

若 Orin 只需尽量少的源码，可在**新克隆**时改用：

```bash
git clone --filter=blob:none --sparse \
  https://github.com/Yuan-Lin-Liu/Coal_FlyCo.git ~/uav/repos/coal_flyco
cd ~/uav/repos/coal_flyco
git sparse-checkout set third_party/flyco_ros2 third_party/ros_interfaces config tools docs
```

完整克隆与稀疏克隆二选一；cone 模式仍可能检出仓库根目录文件。地面机若需语义服务，还要保留 `third_party/SAM_AD_Perception`；选择原生 EVF-SAM 后端时还需 `third_party/EVF-SAM`。

| 源码 | 用途 / 是否保留 |
|---|---|
| `third_party/ros_interfaces/` | 两端自定义消息必须相同；`airsim_ros_pkgs` 在这里仅含消息，名字不代表要安装 AirSim |
| `third_party/flyco_ros2/CMakeLists.txt` | 当前 C++ 编译入口 |
| `vendor/planner_simulator/src/`（在 flyco_ros2 下） | 规划、控制、地图、LKH、kdline；名字含 simulator，不能整目录删除 |
| `vendor/perception_predictor/src/` | 跟踪、点云聚类、预测；按设备选择启动节点 |
| `sim/`、`scenes/`、`assets/`、Isaac/煤堆 USD | 实飞不使用 |
| `bridge/flyco.launch.py`、`bridge/ros_bridge.py` | 当前是仿真入口，不能作为实机 bringup 使用 |

## 5. 编译：哪些现在能做，哪些要先移植

自定义消息可先编译，两端均执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/uav/flight_ws
colcon build --symlink-install --packages-select \
  quadrotor_msgs perception_msgs airsim_ros_pkgs self_msgs_and_srvs \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

**完整 Orin C++ 编译前，必须补齐以下工作；当前仓库未实现自动移植：**

1. `flyco_ros2/CMakeLists.txt` 硬链接了预编译 `libigl.a`、x86-64 的 `libembree3.so`，应改为链接为 arm64 构建且 API 兼容的库；若改用不同 Embree 主版本，还需适配 API。
2. LKH、kdline 和全部本地 `.so/.a` 要在目标架构编译；不能复制桌面机的构建目录。LKH 源码在 `vendor/planner_simulator/src/hierarchical_coverage_planner/solution/LKH/`。
3. 当前 CMake 安装 C++ 节点，但未完整安装 Python 入口、资源和真机 launch；`tools/install_flyco_runtime.py` 是旧 `.local/flyco` 布局的注册办法，不能原样当作标准 colcon 安装。
4. 将求解器绝对路径、每次任务的可写临时目录、相机标定和真实航点生成写入真机配置。当前 launch 会从 Isaac 元数据生成这些参数。`config/flyco.yaml` 中还有 `$(var ...)`、`$(find-pkg-share ...)`，需展开为实际路径，不能直接当作独立节点的最终参数文件。

上述适配完成后，完整编译形式是：

```bash
cd ~/uav/flight_ws
source /opt/ros/humble/setup.bash
source ~/uav/drivers_ws/install/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --packages-up-to flyco_ros2 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 pkg executables flyco_ros2
```

`tools/setup_flyco.sh` 目前只是重编译已有 `.local/flyco` 环境，并非全新 Orin 的安装器。仅设置 `TORCH_CUDA_ARCH_LIST=8.7` 不会自动解决 x86 库、CUDA 和安装布局问题。

## 6. 驱动、定位与 Python 环境

**驱动：** Livox SDK2 在 `~/uav/third_party/Livox-SDK2` 按厂商说明原生编译；ROS 驱动放在 `drivers_ws/src/livox_ros_driver2`。相机和云台型号尚未确定，使用对应厂商 ROS2 驱动。[Livox ROS2 编译说明](https://github.com/Livox-SDK/livox_ros_driver2)。

**定位：** 论文引用 FAST-LIO2，但官方 FAST_LIO 主线仍是 catkin/ROS1。需选定并验证支持 Humble、Mid360 自定义消息的 ROS2 移植，固定其提交号，再放进 `localization_ws/src/`；不能把 ROS1 包直接放入 colcon 后声称编译完成。[官方源码](https://github.com/hku-mars/FAST_LIO)。

**机载 Python：** 即便使用远端语义服务，当前 `SAMTrackingManager` 仍导入 `torch`。不能假定 Orin 只需要一个纯 HTTP 客户端。ROS 节点使用 Python 3.10；用 `venv --system-site-packages` 访问系统 `cv_bridge`，安装与 JetPack 匹配的 Torch，并逐项核对导入。若目标是纯轻量客户端，需要另做依赖拆分。

**地面预测 Python：** 用独立 Python 3.10 venv。依赖版本见仓库 `docker/requirements.txt`（它是可供原生安装使用的依赖清单，不要求 Docker）。还需要 Torch 2.3.1/cu118、torchvision 0.18.1、torch-scatter，以及 NKSR、PointNet2 编译扩展。编译扩展需完整 CUDA Toolkit 与 Torch CUDA 版本兼容。原生安装模板见文末，版本取自当前 `docker/Dockerfile.flyco`。不能只 `pip install -r` 就认为预测环境齐全。

**地面语义服务可直接原生安装**（先准备 Python 3.12；不要替换系统 Python）：

```bash
cd ~/uav/repos/coal_flyco/third_party/SAM_AD_Perception
SAM_AD_PYTHON=python3.12 bash setup_linux.sh A
SAM_AD_A_MODEL=large .venv/bin/python -B -m sam_ad.server \
  --backend A --bind 127.0.0.1 --port 8851
```

此脚本安装桌面 cu128 环境，适用于兼容的地面 NVIDIA 电脑，不是 Orin 安装脚本。当前已验证的是容器中的模型；上述全新原生安装及 Orin 推理仍需目标设备验收。不要用 `tools/semantic_perception_service.py start` 启动原生服务，该管理脚本仍会调用 Docker。

## 7. 必须新增的真机接口

| 适配项 | 现在的情况 | 真机需要实现 |
|---|---|---|
| 控制输出 | `controller.cpp` 把 roll/pitch 角写进 `body_rate`；Isaac 会重新解释 | 选择标准姿态四元数+推力、真实角速度+推力，或另做 PX4 位置指令适配；确认单位、type_mask、坐标变换和推力标定 |
| 状态机 | `PX4CtrlFSM.cpp` 的 RC/模式/超时保护主逻辑被注释 | 恢复模式、遥控接管、odom/cmd 新鲜度检查；不能持续重发过期指令绕过 PX4 心跳保护 |
| 定位输入 | `/isaac/odom` 真值 | FAST-LIO2 输出转换到机体原点；核对位姿、速度各自参考系、TF 和时间戳 |
| PX4 定位融合 | 尚未接入 | 外部定位经 MAVROS vision/odometry 接口送入飞控，正确配置 EKF2 外部定位融合、延迟和协方差 |
| 相机/云台 | `bridge/tracking_node.py` 使用仿真外参和云台符号 | 使用标定后的动态 TF、真实云台反馈和 CameraInfo；RGB+LiDAR 路线需补真实深度/投影链路，不能假设一定有深度相机 |
| 地图 | 传入 `/isaac/lidar/world` 世界点云 | 对齐雷达运动补偿、body/world frame、深度单位，核对 `airsim_flag` 等仿真分支 |
| 任务按钮 | 仿真面板的起飞/Hold/Land 由本地执行器处理 | 提供真实任务触发与飞控模式控制；仿真的 Land 按钮不会自动让 PX4 降落 |

因此，当前没有一个已验证的“实飞运行节点命令”。需要新增 `flyco_real_bringup`，用单独的机载/地面 launch；不能直接运行 `launch_warehouse.sh` 或仅 remap 当前 `px4ctrl_node` 输出。

PX4 外部定位融合与 Offboard 接口按 [v1.16 外部定位文档](https://docs.px4.io/v1.16/en/ros/external_position_estimation)、[Offboard 文档](https://docs.px4.io/v1.16/en/flight_modes/offboard)配置。只给 Orin 的规划器提供 odom，并不等于 PX4 已经融合了定位。

## 8. IP 在哪里改，两台机器怎么通信

举例：Orin 为 `192.168.50.10`，地面电脑为 `192.168.50.20`，两端同一局域网。

| 链路 | 配置位置 | 改什么 |
|---|---|---|
| 雷达 → Orin | 驱动的 `config/MID360_config.json` | `MID360.host_net_info` 中接收数据的各 IP 填 Orin 雷达网口地址；`lidar_configs[].ip` 填实际雷达地址，不是地面电脑地址 |
| Orin → PX4 串口 | MAVROS launch 参数 `fcu_url` | 如 `/dev/ttyACM0:921600`，实际串口/波特率按飞控配置，串口连接没有 IP |
| Orin ↔ 地面预测 | 两端 ROS2 DDS 环境与网络配置 | 相同 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`；`ROS_LOCALHOST_ONLY=0`，接口/路由与组播发现可用 |
| Orin → 语义 HTTP | `/sam_tracking/ros__parameters/segmentation_endpoint` | 旧配置在 `third_party/flyco_ros2/config/flyco.yaml`；真机应放自己的 YAML，当前值 `http://127.0.0.1:8851` |

**预测节点没有硬编码的“电脑 IP”要改。** 它经 DDS 订阅 `/tracking/target_pc`、`/sam_tracking/compress_mask_and_transform`、`/text_prompt` 和真实 CameraInfo，发布 `/prediction/predicted_mesh` 给机载规划器。消息定义要一致，必要话题需跨机可发现。先验证局域网；4G/NAT 需要 VPN 或专门 ROS 网桥，不能只设置一个 IP 就保证连通。当前也没有配置好的跨机限流/压缩方案。

语义服务的原生服务端支持公网卡监听，但要求 token；当前 FlyCo 的 `HTTPTextSAMProvider` 未发送 Authorization。因此**不能只把 127.0.0.1 改成电脑 IP 就断言能运行**。调试时可先用 SSH 隧道，不改服务源码：

```bash
# Orin 上，保持此终端运行；robot 换成地面电脑用户名
ssh -N -T -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8851:127.0.0.1:8851 robot@192.168.50.20
# 机载 segmentation_endpoint 仍保持 http://127.0.0.1:8851
```

需要直接 HTTP 跨机时，再为 Provider 增加与 `SAM_AD_API_TOKEN` 对应的 Bearer header。仿真启动器虽支持 `--segmentation-endpoint`，仍是仿真入口，不能用它替代实机 launch。

机载原始传感器 → 本机定位/控制；目标点云、带位姿图像和文本 → 地面预测；预测网格 → 机载规划。控制循环应独立于预测/网络延迟，预测超时后的行为需在真机适配中定义并测试。

## 9. 实际运行哪些节点、先后顺序是什么

下面是**真机适配完成后的**启动分工；`flyco_real_bringup` 的 launch 目前尚不存在。

| 顺序 | 设备 | 启动内容 | 确认后再继续 |
|---|---|---|---|
| 1 | 飞控板 | 上电，运行已安装的 PX4 v1.16 | 飞控健康、传感器和机架配置正确 |
| 2 | Orin | MAVROS、雷达/相机/云台驱动 | `/mavros/state` 已连接，传感器话题持续发布 |
| 3 | Orin | FAST-LIO2、TF、外部定位送入 PX4 的适配节点 | odom 连续，位姿/速度方向正确，飞控接受定位 |
| 4 | 地面 | 原生语义服务（若放地面）、`mesh_predict_node`、可选 RViz/UI | 模型 ready，ROS2 双向发现和输入输出正常 |
| 5 | Orin | 跟踪 Python 节点、`tracking_prompt`、C++ `sam_tracking_node` | 掩码、目标点云及地面预测网格可见 |
| 6 | Orin | `flyco_planner`、`traj_server`、经修正的 PX4 控制适配 | 航点、控制单位、模式切换和超时保护通过验证 |
| 7 | 操作员 | 经真机任务入口触发初始化、起飞、文本目标和扫描 | 先完成台架/SITL、悬停及遥控接管验证，再执行扫描 |

`rgb_point_mapping_node` 用于彩色建图/评估，按需启动；去掉对 `coal_reference.ply` 真值的依赖。`joy_cmd` 仅在选择其任务触发方案时启动，避免与轨迹节点同时争抢位置指令。

可先做的真实 MAVROS 连通检查（不会自动解锁或起飞）：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
# 另一已加载 ROS 的终端
ros2 topic echo /mavros/state --once
ros2 topic hz /mavros/imu/data
```

正式飞行前应确认：没有依赖 `/clock`，各节点 `use_sim_time=false`；定位坐标、传感器外参和数据 QoS 正确；地面断网或模型退出不影响 PX4/机载保护；降落、停桨和退出顺序已测试。Orin 型号/内存、相机/云台和选用的 FAST-LIO2 ROS2 版本未确定，因此当前不能提供一套已验证的自动起飞命令。


## 附：地面预测模块原生依赖安装模板

在地面 x86 电脑执行。前提是已经装好 ROS2 Humble、系统 Python 3.10、CUDA Toolkit 11.8 和编译工具；下列命令是从已用镜像构建配方整理出的原生模板，全新原生安装仍需验收。语义服务继续用第 6 节的独立 Python 3.12 环境。

```bash
REPO="$HOME/uav/repos/coal_flyco"
PRED_ENV="$REPO/.local/flyco/venv"
python3.10 -m venv --system-site-packages "$PRED_ENV"
"$PRED_ENV/bin/python" -m pip install 'pip<25' setuptools==69.5.1 wheel
"$PRED_ENV/bin/python" -m pip install torch==2.3.1 torchvision==0.18.1 \
  --index-url https://download.pytorch.org/whl/cu118
"$PRED_ENV/bin/python" -m pip install -r "$REPO/docker/requirements.txt"
"$PRED_ENV/bin/python" -m pip install --no-index --no-deps \
  -f https://data.pyg.org/whl/torch-2.3.0+cu118.html torch-scatter==2.1.2+pt23cu118

# 以下路径用于首次安装；如果已有 NKSR，先核对版本，不覆盖原目录。
NKSR_SRC="$HOME/uav/third_party/NKSR"
mkdir -p "$NKSR_SRC"
git -C "$NKSR_SRC" init
git -C "$NKSR_SRC" remote add origin https://github.com/nv-tlabs/NKSR.git
git -C "$NKSR_SRC" fetch --depth 1 origin e40336845e67761343a756788e5a98b827d4a143
git -C "$NKSR_SRC" checkout FETCH_HEAD
mkdir -p "$NKSR_SRC/package/external/openvdb"
git -C "$NKSR_SRC/package/external/openvdb" init
git -C "$NKSR_SRC/package/external/openvdb" remote add origin \
  https://github.com/AcademySoftwareFoundation/openvdb.git
git -C "$NKSR_SRC/package/external/openvdb" fetch --depth 1 origin \
  7edd8cd86f105a01a41ad7b2bf59a81034cd79fb
git -C "$NKSR_SRC/package/external/openvdb" checkout FETCH_HEAD
git clone --depth 1 --branch 3.4 https://gitlab.com/libeigen/eigen.git \
  "$NKSR_SRC/package/external/eigen"
export CUDA_HOME=/usr/local/cuda-11.8
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
export MAX_JOBS=2
"$PRED_ENV/bin/python" -m pip install --no-build-isolation "$NKSR_SRC/package"

# 准备预测权重和固定 Hugging Face 缓存；已有通过校验的文件会复用。
"$PRED_ENV/bin/python" "$REPO/tools/download_models.py" --huggingface
MESH="$REPO/third_party/flyco_ros2/vendor/perception_predictor/src/mesh_predict"
export PYTHONPATH="$REPO/third_party/flyco_ros2/python:$MESH/src:${PYTHONPATH:-}"
export TORCH_EXTENSIONS_DIR="$REPO/.local/flyco/cache/torch_extensions"
mkdir -p "$TORCH_EXTENSIONS_DIR"
"$PRED_ENV/bin/python" -c 'import torch,nksr,pointnet2_ops; assert torch.cuda.is_available()'
```

最后一行会触发 PointNet2 CUDA 扩展编译。两端自定义消息编译完成后，地面节点应使用真实的 CameraInfo、位姿和任务参数启动；不要沿用仿真相机数值、`use_sim_time=true` 或真值参考模型。网络传输的输入输出见第 8 节，统一启动入口仍待实现。
