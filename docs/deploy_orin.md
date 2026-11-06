# Orin + Odin + PX4 部署

分支 `dev-real`，ROS2 Humble。先在仿真验证，再接入真机。PX4 自定义固件必须使用匹配的 `px4_msgs`；仓库默认消息对应 PX4 v1.16，支持外部模式的 QGC 版本才能显示 FlyCo。当前本机 SITL 是定制固件，仿真时另加载其匹配的消息覆盖层。

论文 §7.1.1 的分工：**地面电脑运行表面预测；Orin 运行定位、感知、规划和轨迹执行**。原文使用 Orin NX、Mid360 和云台相机；本工程以 Odin 设备内 SLAM 替换定位链，固定相机使用实测外参。来源：[FlyCo 论文](https://arxiv.org/html/2601.07558v1#S7.SS1.SSS1)。

| 设备 | 模块 |
|---|---|
| Odin | 设备端 SLAM；输出位姿、点云、IMU、RGB |
| Orin | Odin 驱动、坐标/时间适配、语义感知、点云聚类、FlyCo 规划、轨迹服务、PX4 FlyCo 模式 |
| 地面电脑 | 网格预测、文字提示界面、RViz2、QGC |
| PX4 | 外部定位融合、位置/姿态控制、电机分配、模式和失效处理 |

## 1. 依赖与编译

默认两端已经配置 ROS2、CUDA、编译器。额外依赖：colcon/ament、PCL、Eigen3、OpenCV/cv_bridge、OMPL、yaml-cpp、Embree **3**（Orin 必须为 ARM64 构建）、Python NumPy/SciPy、匹配固件的 MicroXRCEAgent。Odin 需要 libusb、OpenCV 及仓库 SDK 所列运行库。模型环境需匹配 JetPack/CUDA 的 PyTorch、SAM 感知依赖；地面预测另需 NKSR、PointNet2 CUDA 扩展及仓库清单中的权重。不要将 x86 虚拟环境或 `.so` 复制到 Orin。

模型清单：[预测权重](../config/models.json)、[语义资产](../third_party/SAM_AD_Perception/assets.lock.json)、[语义服务 Python 依赖](../third_party/SAM_AD_Perception/requirements-server.txt)。服务依赖清单来自桌面环境；Orin 的 PyTorch/CUDA 和 Python 依赖版本需匹配 JetPack，不照搬桌面的 cu128 环境。地面预测 Python 环境须兼容 ROS2 Humble 的 Python 3.10，并能导入 `rclpy`、`cv_bridge`；语义服务通过 HTTP 连接，可使用独立模型环境。

两端拉取已发布的分支并创建各自的 Odin 配置；保留原有 `vehicle.local.yaml`：

```bash
git clone --branch dev-real https://github.com/Yuan-Lin-Liu/Coal_FlyCo.git coal_flyco
cd coal_flyco
cp -n ros2/flyco_real/config/odin.yaml config/vehicle.odin.local.yaml
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
```

Orin 编译：

```bash
COLCON_DEFAULTS_FILE="$PWD/config/colcon_onboard.yaml" colcon --log-base runtime/native/onboard/log build
```

地面编译：

```bash
COLCON_DEFAULTS_FILE="$PWD/config/colcon_ground.yaml" colcon --log-base runtime/native/ground/log build
```

若 Embree 不在系统搜索路径，编译前设置 `CMAKE_PREFIX_PATH` 指向目标架构安装目录。更新后重复对应构建命令。当前 `dev-real` 尚为本地分支，远端拉取命令须在分支发布后使用。

## 2. 配置与每个终端的环境

Orin 每个终端：

```bash
source /opt/ros/humble/setup.bash
source runtime/native/onboard/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

地面每个终端将 `onboard` 改成 `ground`。两端与 PX4 的 `UXRCE_DDS_DOM_ID` 保持一致，两端系统时间同步，DDS 网络互通。

编辑 `config/vehicle.odin.local.yaml`：

- 填写 `odin_adapter.ros__parameters.body_to_imu_*` 的实测安装外参。
- 确认图像已去畸变、相机光学坐标系和 CameraInfo 一致。
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
