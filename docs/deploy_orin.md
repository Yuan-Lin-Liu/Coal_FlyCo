# Orin + Odin + PX4 部署

分支 `dev-real`，ROS2 Humble。

论文 §7.1.1 的分工：**地面电脑运行表面预测；Orin 运行定位、感知、规划和轨迹执行**。
原文使用 Orin NX、Mid360 和云台相机；本工程以 Odin 设备内 SLAM 替换定位链，固定相机使用实测外参。

| 设备 | 模块 |
|---|---|
| Odin | 设备端 SLAM；输出位姿、点云、IMU、RGB |
| Orin | Odin 驱动、坐标/时间适配、语义感知、点云聚类、FlyCo 规划、轨迹服务、PX4 FlyCo 模式 |
| 地面电脑 | 网格预测、文字提示界面、RViz2、QGC |
| PX4 | 外部定位融合、位置/姿态控制、电机分配、模式和失效处理 |

## 1. 依赖与编译

默认两端已经配置 ROS2、CUDA、编译器。额外依赖：colcon/ament、`colcon-defaults`、Ninja、PCL、Eigen3、OpenCV/cv_bridge、OMPL、yaml-cpp、Embree **3**（Orin 必须为 ARM64 构建）、Python NumPy/SciPy、匹配固件的 MicroXRCEAgent。Odin 需要 libusb、OpenCV 及仓库 SDK 所列运行库。模型环境需匹配 JetPack/CUDA 的 PyTorch、SAM 感知依赖；地面预测另需 NKSR、PointNet2 CUDA 扩展及仓库清单中的权重。不要将 x86 虚拟环境或 `.so` 复制到 Orin。

模型清单：[预测权重](../config/models.json)、[语义资产](../third_party/SAM_AD_Perception/assets.lock.json)、[语义服务 Python 依赖](../third_party/SAM_AD_Perception/requirements-server.txt)。服务依赖清单来自桌面环境；Orin 的 PyTorch/CUDA 和 Python 依赖版本需匹配 JetPack，不照搬桌面的 cu128 环境。地面预测 Python 环境须兼容 ROS2 Humble 的 Python 3.10，并能导入 `rclpy`、`cv_bridge`；语义服务通过 HTTP 连接，可使用独立模型环境。

对于当前部署的 Jetson Linux **R36.4.0 / JetPack 6.1 / Python 3.10**，先通过已配置的
R36.4 NVIDIA 软件源安装计算组件：

```bash
sudo apt install -y --no-install-recommends cuda-toolkit-12-6 \
  libcudnn9-cuda-12 libcudnn9-dev-cuda-12 python3-venv python3-dev libopenblas-dev
```

然后在 Orin 的项目根目录安装语义模型环境：

```bash
bash tools/setup_semantic_jetson.sh
```

该入口使用 Jetson AI Lab `jp6/cu126` 的 ARM64 PyTorch 2.8.0 / TorchVision 0.23.0
配对安装包，校验安装包哈希，在 `third_party/SAM_AD_Perception/.venv` 创建 Python 3.10
环境并准备后端 A 的模型资产。Python 3.10 使用 scikit-image 0.25.2；推理服务采用
headless OpenCV，UI 继续使用地面环境。桌面依赖清单保持原样。安装过程需要访问
清华 PyPI 镜像、Jetson AI Lab、GitHub 和模型资产下载源；已有有效资产会复用。
普通 Python 包默认走清华镜像，可用 `SAM_AD_PIP_INDEX_URL` 指定其他索引。
引导升级 pip 时绕过 HTTP 缓存，后续使用新版 pip 的下载重试与续传；所有哈希校验保留。
若下载中断后提示哈希不一致，可更新代码后重跑同一安装命令，已有虚拟环境和有效资产会复用。

若 Jetson AI Lab 的 PyTorch 下载过慢，可在网络较快的电脑下载脚本中固定 URL 对应的
`torch-2.8.0-cp310-cp310-linux_aarch64.whl` 和
`torchvision-0.23.0-cp310-cp310-linux_aarch64.whl`，通过局域网或 U 盘传入 Orin 的
`.local/deploy/jetson-wheels/` 后重跑脚本。脚本优先使用这些本地文件，仍按原 SHA256 校验；
缺少的文件继续从原地址下载。也可通过 `SAM_AD_WHEEL_DIR` 指定存放目录。
清华镜像仅加速普通 Python 包，不会替换脚本指定的 Jetson GPU 包直链。
这两个包仅解决 PyTorch 安装，后续模型权重仍需下载。

安装完成后直接启动 `ros2 launch flyco_real semantic.launch.py`，由服务加载模型并完成
GPU 预热。启动服务无需激活 `.venv`，也无需重新编译 ROS2 包。该 ARM64 安装路径已核对
轮子元数据、CUDA 12.6 链接依赖和脚本语法；本机 x86_64 无法替代 Orin 的实际 GPU 启动验收。

两端拉取已发布的分支并创建各自的 Odin 配置；保留原有 `vehicle.local.yaml`：

```bash
git clone --branch dev-real https://github.com/Yuan-Lin-Liu/Coal_FlyCo.git coal_flyco
cd coal_flyco
cp -n ros2/flyco_real/config/odin.yaml config/vehicle.odin.local.yaml
source /opt/ros/humble/setup.bash
```

**Orin 和地面电脑使用同一条编译命令**，在工程根目录执行：

```bash
colcon build
```

编译结果为根目录的 `build/`、`install/`、`log/`。根目录 `colcon_defaults.yaml` 由 colcon 自动读取，限定 ROS2 包的搜索范围并限制并行编译内存；无需设置 `COLCON_DEFAULTS_FILE`。默认编译 8 个 ROS2 包，包括 Odin 驱动、PX4 消息、FlyCo 算法和真机入口。模块分工由下方各设备的 launch 决定。

Odin SDK 按目标架构选择；Orin 使用目标架构安装的 Embree 3 和 OMPL，x86 仿真主机可复用现有依赖缓存。非系统依赖前缀仍按安装位置设置 `CMAKE_PREFIX_PATH`。更新代码后再次 `colcon build` 即可。构建结构、单包维护和改造前 HEAD 见 [构建说明](build.md)。

### PX4 固件与消息版本

工程内 `third_party/px4_msgs` 已从本机 `/home/robot/PX4-Autopilot` 同步：
`NextFlightLab/PX4-Autopilot`、`feature/neural`、提交
`0d092a91f0c0417ab888b4bc37331484e6d46b9d`，出处见
[SOURCE.json](../third_party/px4_msgs/SOURCE.json)。注册请求/回复、解锁检查请求/回复、
本地位置采用版本 1；节点根据消息常量自动使用 `_v1` 话题。PX4 ROS2 模式 API
版本仍是消息定义中的 `1`，它与固件版本号 `1.17` 是不同的编号。

已部署的旧工作区更新后，停止旧的 FlyCo ROS2 节点，在 Orin 和地面电脑各自执行一次增量编译：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs flyco_real --cmake-force-configure
source install/setup.bash
```

不需要重编已通过的 `flyco_ros2` 算法。所有运行终端重新加载更新后的环境，避免继续导入
旧工作区的 `px4_msgs`。保持与飞控一致的 ROS 域；例如已使用域 66 的现场终端继续使用 66。

Agent 与飞控 DDS client 已连接后，直接启动注册与使能节点：

```bash
ros2 launch flyco_real mode.launch.py
```

看到 `Registered FlyCo: nav_state=...` 表示模式注册成功；定位、地面心跳和解锁条件
继续由原有检查决定。注册失败时，在另一终端运行 `ros2 run flyco_real task.py status`，
查看 `registration_issue`。实机报告的 `feature/vv_agentic` / `f026a1d...` 与同步源码提交不同，
若出现消息版本问题，应使用实际编译该固件的消息定义，不能只修改话题后缀。
若本地位置的 `ref_alt` 为 NaN，当前原生 Takeoff 入口会拒绝自动起飞，需要先完成高度参考配置。

## 2. 每个终端的环境与设备配置

Orin 和地面电脑每个终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

ROS2 域、网络范围和 RMW 实现暂不指定，真机部署时根据地面电脑、Orin 和现有 PX4 DDS 配置统一确认。若现有通信已正常，直接复用；需要调整时再确定 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、`RMW_IMPLEMENTATION` 及飞控的 `UXRCE_DDS_DOM_ID`。部署时还需保证两端时间同步和 DDS 网络互通。仿真继续按 [仿真说明](px4_qgc.md) 使用域 64。

**Odin 自带的标定由驱动处理**：驱动连接设备时读取该设备的 `calib.yaml`，根据标定生成去畸变图像。本工程已默认启用 `/odin1/image/undistorted`，并由驱动发布匹配的 `/odin1/camera_info`，两者使用 `odin_camera_optical_frame`；设备内部的 IMU、雷达、相机外参通过 `/odin1/wiwc` 交给适配节点。正常使用本工程提供的驱动和配置，无需手工填写相机内参、重复去畸变或重新标定设备内部外参。驱动原说明见 [标定文件](../third_party/drivers/odin_ros_driver/README.md#41-file-structure) 和 [话题说明](../third_party/drivers/odin_ros_driver/README.md#43-ros-topics)；具体话题名以本工程驱动实现为准。

`odin_adapter.ros__parameters.body_to_imu_*` 是 **Odin 在飞机上的安装关系**，设备出厂标定无法知道它：

- `body_to_imu_translation`：Odin IMU 原点在飞机 `base_link`（前、左、上）坐标系中的位置，单位米。
- `body_to_imu_quaternion`：将 Odin IMU 坐标中的向量旋转到飞机机体系的旋转，顺序 `[x, y, z, w]`。

这两项现在先保留模板，等实际机架、安装朝向和原有定位链确认后再处理。如果此前已通过 TF、机体原点定义或 PX4 外部定位参数处理过安装关系，应先核对并复用已有结果，避免重复补偿。模板中的零平移和单位四元数表示两坐标系完全重合，只有实际采用该定义时才适用。图像、CameraInfo 和光学坐标的一致性也在设备接入时确认，正常情况下由上述驱动自动保证；更换驱动版本、图像分辨率或另接相机后才需要重新核对相应配置。

其余 `config/vehicle.odin.local.yaml` 配置在真机部署时确认：

- Orin 的 `segmentation_endpoint` 保持 `http://127.0.0.1:8851`，语义推理在 Orin。
- 地面模型 Python、权重路径通过下面的 launch 参数指定。
- 按实机配置 `bounds_min/max`、速度、加速度和飞控外部定位融合。定位输入只保留一个发布源。

PX4 外部视觉融合参数按 [官方外部定位说明](https://docs.px4.io/v1.16/en/ros/external_position_estimation) 配置；本工程输出 FRD 参考的外部位姿，不宣称提供已经验证的外部速度。

## 3. 按顺序启动

**Orin 终端 A：飞控 DDS 链路**（已有 Agent 时复用，不重复启动）：

```bash
ros2 launch flyco_real agent.launch.py transport:=serial device:=/dev/ttyACM0 baudrate:=921600
```

**Orin 终端 B：Odin 独立定位**：

```bash
ros2 launch flyco_real odin.launch.py config:="$PWD/config/vehicle.odin.local.yaml"
```

此入口运行 Odin 驱动、安装标定适配和传感器桥，可独立使用。数据转换至 `/flyco/odom`、`/flyco/lidar/world`、`/flyco/camera/rgb`，并发送 PX4 外部视觉位姿。

**Orin 终端 C：本机语义推理**：

```bash
ros2 launch flyco_real semantic.launch.py python:=/实际Orin模型环境/bin/python
```

如启用语义 API token，在 Orin 的服务与算法终端设置相同 `SAM_AD_API_TOKEN`。

**地面终端 A：预测、提示界面与 RViz2**：

```bash
ros2 launch flyco_real ground.launch.py config:="$PWD/config/vehicle.odin.local.yaml" \
  model_python:=/实际地面模型环境/bin/python models_dir:=/实际权重目录 rviz:=true
```

**Orin 终端 D：注册与自动使能**：

```bash
ros2 launch flyco_real mode.launch.py config:="$PWD/config/vehicle.odin.local.yaml"
```

定位、飞控、地面心跳稳定后，节点自动注册并使能 FlyCo；不会自动解锁。

**Orin 终端 E：FlyCo 机载算法与任务节点**：

```bash
ros2 launch flyco_real flyco.launch.py config:="$PWD/config/vehicle.odin.local.yaml"
```

此入口复用独立 Odin 定位桥，不再创建第二个定位桥或模式节点；地面预测继续由地面 launch 运行。

## 4. 飞行操作

等待机载终端出现 `FlyCo ready`，确认定位和地面预测链路正常。在地面以 **Position** 模式正常解锁，然后通过 RC/QGC 切换到 **FlyCo**。任务节点自动调度 PX4 内置 Takeoff，升至距本次地面位置约 2 m 后返回 FlyCo 悬停。起飞只触发一次；空中切模式、未解锁或落地状态未知时不自动起飞。

需要单独打开文字输入窗口时，在地面电脑执行 `ros2 launch flyco_real prompt.launch.py`，在 **FlyCo-UI** 底部 Text 输入英文描述，点击 **Send text** 发送。已有地面 UI 时不重复打开。

起飞悬停后发送目标文字，等待目标点云及预测网格，再激活规划和扫描：

```bash
ros2 run flyco_real task.py text '一个煤堆'
ros2 run flyco_real task.py enable-planning
ros2 run flyco_real task.py scan
```

按需要执行对应操作：

```bash
ros2 run flyco_real task.py hold
ros2 run flyco_real task.py resume
ros2 run flyco_real task.py land
ros2 run flyco_real task.py status
```

任务状态为 `/flyco/mission/status`。降落请求成功后仍需确认实际落地、上锁、停桨，再结束节点。`task.py enable/select-mode/arm/takeoff` 保留为显式操作工具，常规流程不需要逐条执行。

## 5. 地面 RViz2 与核心排查

地面 `ground.launch.py rviz:=true` 自动载入真机配置，固定坐标系为 `world`，观察机体姿态、点云、图像、预测和轨迹。红色相机视锥由 `/flyco/camera/fov` 根据实际标定和光学坐标 TF 显示，独立于是否已生成轨迹。手动打开时：

```bash
ros2 run rviz2 rviz2 -d "$PWD/ros2/flyco_real/config/flyco_real.rviz"
```

仅需优先检查三个话题：`/flyco/odom`（定位）、`/flyco/control/status`（链路/模式/控制状态）、`/prediction/predicted_mesh`（地面预测）。点云为 `/flyco/lidar/world`，图像为 `/flyco/camera/rgb`。无显示时先查 ROS 域、网络、时间、TF，再看对应终端首个错误。

## 验证范围

原 PX4 仿真已验证扫描、QGC 实际界面降落和 RViz 显示。原生 FlyCo 模式已完成正常解锁、Takeoff → FlyCo、预测、完整扫描及落地停桨；ULog 确认全程未进入 Offboard。地面包、Odin 驱动和完整机载算法均在 x86_64 编译通过，新入口完成参数解析。具体证据见 [验收记录](dev-real-status.md)。当前没有连接 Orin/Odin 实机，ARM64 构建、双机网络、设备标定和实飞仍需硬件验收。
