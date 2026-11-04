# Orin 真机部署指南

本文针对 Jetson Orin（按论文中的 Orin NX、Livox Mid360、机载相机和 PX4 飞控）说明部署流程。当前仓库中的 Isaac Sim、Pegasus 和 `launch_warehouse.sh` 只用于仿真；它们不能直接在 Orin 真机上运行。

论文的实机方案把预测模块放在地面 RTX 计算机，其余感知、定位、规划和控制模块运行在 Orin 上。
## 1. 准备 Orin

建议使用 JetPack 6.x、Ubuntu 22.04 arm64、ROS 2 Humble。先确认架构和 JetPack：

```bash
uname -m                         # 必须是 aarch64
cat /etc/nv_tegra_release       # 查看 JetPack/L4T 版本
sudo nvpmodel -q
sudo jetson_clocks                # 仅在测试或任务前按需启用
```

JetPack、CUDA 和 TensorRT 必须使用与当前 L4T 匹配的版本。不要在 Orin 上执行本项目的 `docker/setup.sh`：其中的 Isaac 镜像和 FlyCo 镜像按 x86_64/桌面 GPU 构建，且 Dockerfile 固定了 CUDA 11.8 与 x86 PyTorch wheel。

安装 ROS 2 Humble 和编译工具。

## 2. 拉取代码

```bash
sudo mkdir -p /opt
sudo chown "$USER:$USER" /opt
git clone --branch main git@github.com:Yuan-Lin-Liu/Coal_FlyCo.git /opt/coal_flyco
cd /opt/coal_flyco
```

如果 Orin 没有 GitHub SSH 密钥，把地址换成：

```bash
git clone --branch main https://github.com/Yuan-Lin-Liu/Coal_FlyCo.git /opt/coal_flyco
```

## 3. 准备模型和 Python

先确定感知和预测放在哪里：

| 模块 | 推荐位置 | 说明 |
|---|---|---|
| SAM/语义感知 | Orin 或地面 GPU | Orin 需使用 Jetson 兼容的 PyTorch/TensorRT；地面运行时通过网络服务访问 |
| 网格预测 | 地面 GPU | 论文明确将计算密集的预测放在地面计算机 |
| 规划、控制、点云处理 | Orin | 需要低延迟和稳定的 ROS 话题 |

本项目的模型下载脚本只负责下载和校验文件，不能解决 Orin 的 CUDA/PyTorch ABI：

```bash
mkdir -p .local/flyco/models
python3 tools/download_models.py --check --directory .local/flyco/models
```

如果模型放在地面计算机，Orin 只保留客户端，并把服务地址改成地面机 IP；不要把服务绑定在 `127.0.0.1`。语义感知服务的默认地址是 `http://127.0.0.1:8851`，真机应改成例如 `http://192.168.1.20:8851`，并在地面机防火墙中只开放必要端口。

## 4. 编译 FlyCo C++ 部分

先安装本机 ROS 消息依赖：

```bash
sudo apt install -y \
  ros-humble-mavros-msgs ros-humble-geographic-msgs \
  ros-humble-pcl-conversions ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-message-filters \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-rviz2
```

然后编译自定义消息和 FlyCo：

```bash
source /opt/ros/humble/setup.bash
export FLYCO_RUNTIME=/opt/coal_flyco/.local/flyco
export FLYCO_CUDA_HOME=/usr/local/cuda
export TORCH_CUDA_ARCH_LIST=8.7       # Orin 的 CUDA 架构
export BUILD_JOBS=2

# 仅当已准备好 Jetson 兼容的 venv、Torch 和依赖后执行
./tools/setup_flyco.sh
source tools/flyco_env.sh
```

在执行编译前必须完成以下三项适配，否则不能认为编译结果可用于飞行：

1. 将 `tools/flyco_env.sh` 中桌面机专用的 CUDA 11.8、`x86_64-linux-gnu` 和 gcc-11 路径改为 Orin 的 `/usr/local/cuda`、arm64 库路径和实际编译器。
2. 重新编译仓库中 `third_party/flyco_ros2/vendor/planner_simulator/src/3rdparty/embree-3` 和 `libigl`；仓库中的预编译库不能直接假定兼容 arm64。
3. 为 Orin 安装与 JetPack 匹配的 PyTorch，并重新编译 `sam2/_C.so`、PointNet2 CUDA 扩展。不要直接安装 Dockerfile 中的 x86 CUDA 11.8 wheel。可参考 [NVIDIA Jetson PyTorch 文档](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)。

## 5. 接入真实传感器和定位

论文实机使用 Livox Mid360、IMU 和云台相机；你的定位节点需要发布 FlyCo 所需的 ENU/FLU 坐标消息。建议先用 rosbag 验证，再接飞控：

```text
定位节点       -> /flyco/odom       nav_msgs/Odometry
IMU            -> /flyco/imu        sensor_msgs/Imu
Livox 点云     -> /flyco/lidar      sensor_msgs/PointCloud2
RGB 相机       -> /flyco/camera/rgb  sensor_msgs/Image
深度相机       -> /flyco/camera/depth sensor_msgs/Image
相机内参       -> /flyco/camera/info sensor_msgs/CameraInfo
```

当前仿真 launch 使用 `/isaac/odom`、`/isaac/imu`、`/isaac/camera/*` 和 `/isaac/lidar/world`。真机 launch 必须把这些 remap 到实际驱动话题，并提供相机、雷达、机体和云台的准确外参。不要直接把 Isaac 的 `bridge/ros_bridge.py` 用在真机上，因为它会发布仿真状态并接管仿真时间。

定位节点至少要满足：

- 输出稳定的 `nav_msgs/Odometry`，坐标方向、单位和时间戳明确；
- 里程计、IMU、点云和图像使用同一时钟，时间戳单调递增；
- 发布的机体姿态和相机/雷达外参与 `config/coal_warehouse.yaml` 的相机模型一致；
- 先静态运行检查 10 分钟，再进行手动低速飞行测试。

## 6. 连接 PX4 v1.16

先给 PX4 飞控建立 MAVROS 链路。串口路径和波特率以你的飞控配置为准：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch \
  fcu_url:=/dev/ttyACM0:921600 \
  gcs_url:=udp://@192.168.1.20:14550
```

验证链路：

```bash
ros2 topic echo /mavros/state --once
ros2 topic hz /mavros/imu/data
ros2 topic echo /mavros/local_position/odom --once
```

本项目的 `px4ctrl_node` 接收 odom、IMU 和位置指令，并发布 `mavros_msgs/msg/AttitudeTarget`。MAVROS 的真实输入话题是 `/mavros/setpoint_raw/attitude`；仿真代码默认发布 `/setpoint_raw/attitude`，因此真机 launch 必须加 remap 或修改控制节点：

```bash
ros2 run flyco_ros2 px4ctrl_node --ros-args \
  -r /px4ctrl/odom:=/flyco/odom \
  -r /px4ctrl/imu:=/flyco/imu \
  -r /setpoint_raw/attitude:=/mavros/setpoint_raw/attitude
```

还要把 `px4ctrl_node` 的 `/px4ctrl/cmd` 接到 FlyCo planner 的位置指令，并确认 `mavros/set_mode` 服务和 PX4 的 Offboard 配置可用。PX4 要求进入 Offboard 前持续发送 setpoint/心跳，且必须配置失联保护；参见 [PX4 v1.16 Offboard 文档](https://docs.px4.io/v1.16/en/flight_modes/offboard.html)。

## 7. 真机启动顺序

建议拆成 4 个终端或 systemd 服务：

```bash
# 终端 1：ROS 环境
source /opt/ros/humble/setup.bash

# 终端 2：飞控链路
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600

# 终端 3：定位、Livox、相机驱动
ros2 launch <your_lio_package> <your_lio_launch>.py
ros2 launch <your_sensor_package> <your_sensor_launch>.py

# 终端 4：真实传感器版 FlyCo bringup
cd /opt/coal_flyco
source tools/flyco_env.sh
ros2 launch <your_flyco_real_package> bringup_real.launch.py
```

仓库目前没有 `bringup_real.launch.py`；需要新建一个真机 launch，至少启动 `flyco_planner`、`traj_server`、`px4ctrl_node`、`rgb_point_mapping_node`、`sam_tracking_node` 和 `mesh_predict_node`，并完成第 5 节的所有 remap。`prompt_ui` 和 RViz 可放在地面站，不建议作为飞行控制链路的必要节点。

## 8. 上电前验收

```bash
ros2 topic list | rg 'mavros|flyco|camera|livox|odom|imu'
ros2 topic hz /flyco/odom
ros2 topic hz /flyco/imu
ros2 topic hz /flyco/camera/rgb
ros2 topic hz /flyco/lidar
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/setpoint_raw/target_attitude --once
```

必须先通过“无桨台架测试”和 PX4 SITL/HITL，再进行系留起飞。首次真机只验证悬停、Hold 和 Land；确认 Offboard 失联时 PX4 能进入预期 failsafe 后，再启用自动扫描。

## 9. 常见错误

| 现象 | 原因 |
|---|---|
| `docker/setup.sh` 构建失败 | 当前 Dockerfile 是 x86_64/桌面 CUDA 配置，不是 Jetson L4T 配置 |
| `torch.cuda.is_available()` 为 `False` | JetPack、PyTorch wheel 和 CUDA 不匹配 |
| `px4ctrl_node` 有输出但 PX4 不动 | 输出仍在 `/setpoint_raw/attitude`，没有 remap 到 `/mavros/setpoint_raw/attitude`，或 PX4 未进入 Offboard |
| 规划器报无 odom/点云 | 真机话题没有 remap，或坐标/时间戳不一致 |
| 预测节点拖慢飞行控制 | 预测应迁移到地面 GPU；控制、定位和安全链路不能等待预测结果 |

论文和仓库链接：

- [FlyCo 论文实机平台](https://arxiv.org/html/2601.07558v1)
- [Coal_FlyCo 仓库](https://github.com/Yuan-Lin-Liu/Coal_FlyCo)
