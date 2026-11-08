# Orin + Odin + PX4 部署

分支 `dev-real`，ROS2 Humble。

从两台新设备开始准备时，完整依赖、架构说明和按终端排列的命令见 [真机双机部署指令](dual_host_deployment.md)；其中单独列出了尚待 Orin/固件信息确认的移植条件。

论文 §7.1.1 的分工：**地面电脑运行表面预测；Orin 运行定位、感知、规划和轨迹执行**。
原文使用 Orin NX、Mid360 和云台相机；本工程以 Odin 设备内 SLAM 替换定位链，直接复用原驱动的相机 TF，Odin IMU 与机体按零安装偏移处理。

| 设备 | 模块 |
|---|---|
| Odin | 设备端 SLAM；输出位姿、点云、IMU、RGB |
| Orin | Odin 驱动、坐标/时间适配、语义感知、点云聚类、FlyCo 规划、轨迹服务、PX4 FlyCo 模式 |
| 地面电脑 | 网格预测、文字提示界面、RViz2、QGC |
| PX4 | 外部定位融合、位置/姿态控制、电机分配、模式和失效处理 |

## 1. 依赖与编译

默认两端已经配置 ROS2、CUDA、编译器。额外依赖：colcon/ament、`colcon-defaults`、Ninja、PCL、Eigen3、OpenCV/cv_bridge、OMPL、yaml-cpp、Embree **3**（Orin 必须为 ARM64 构建）、Python NumPy/SciPy、匹配固件的 MicroXRCEAgent。Odin 需要 libusb、OpenCV 及仓库 SDK 所列运行库。模型环境需匹配 JetPack/CUDA 的 PyTorch、SAM 感知依赖；地面预测另需 NKSR、PointNet2 CUDA 扩展及仓库清单中的权重。不要将 x86 虚拟环境或 `.so` 复制到 Orin。

模型清单：[预测权重](../config/models.json)、[语义资产](../third_party/SAM_AD_Perception/assets.lock.json)、[语义服务 Python 依赖](../third_party/SAM_AD_Perception/requirements-server.txt)。服务依赖清单来自桌面环境；Orin 的 PyTorch/CUDA 和 Python 依赖版本需匹配 JetPack，不照搬桌面的 cu128 环境。地面预测 Python 环境须兼容 ROS2 Humble 的 Python 3.10，并能导入 `rclpy`、`cv_bridge`；语义服务通过 HTTP 连接，可使用独立模型环境。

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

## 2. 每个终端的环境与设备配置

Orin 和地面电脑每个终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

ROS2 域、网络范围和 RMW 实现暂不指定，真机部署时根据地面电脑、Orin 和现有 PX4 DDS 配置统一确认。若现有通信已正常，直接复用；需要调整时再确定 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、`RMW_IMPLEMENTATION` 及飞控的 `UXRCE_DDS_DOM_ID`。部署时还需保证两端时间同步和 DDS 网络互通。仿真继续按 [仿真说明](px4_qgc.md) 使用域 64。

**Odin 自带的标定由原驱动处理**：驱动读取设备的 `calib.yaml`，输出 `/odin1/image/undistorted`、匹配的 `/odin1/camera_info`，以及原生 `odom → imu`、`imu → lidar`、`odom → camera_0` TF。无需额外外参适配节点或 `/odin1/wiwc` 订阅。

本工程默认 IMU 与机体原点、轴向重合，直接转发机体位姿，不发布安装外参 TF、不做安装偏移补偿。保留 `world → odom` 世界系连接、`world → base_link` 机体位姿和原驱动 TF。桥接后的 RGB、CameraInfo 使用原驱动光学 TF 名 `camera_0`（原始图像头为 `odin_camera_optical_frame`），仅统一同一光学坐标系的名称，保留内参和图像数据。ROS ENU/FLU 到 PX4 坐标约定的转换继续保留。

已有车辆配置中的旧 `odin_adapter` 字段会被忽略，Odin 启动入口自动启用原驱动 TF。图像、CameraInfo 和光学 TF 仍应来自同一设备及匹配驱动版本。

若通过 SCP/压缩包覆盖旧工程，覆盖操作不会删除旧文件。在 Orin 工程根目录清理已停用的适配节点后，仅重编译真机入口包即可；无需清空其他编译产物或车辆配置：

```bash
rm -f ros2/flyco_real/flyco_real/odin_adapter.py \
  ros2/flyco_real/scripts/odin_adapter.py \
  install/flyco_real/lib/flyco_real/odin_adapter.py \
  install/flyco_real/local/lib/python3.10/dist-packages/flyco_real/odin_adapter.py
source /opt/ros/humble/setup.bash
colcon build --packages-select flyco_real
source install/setup.bash
```

其余 `config/vehicle.odin.local.yaml` 配置在真机部署时确认：

- Orin 的 `segmentation_endpoint` 保持 `http://127.0.0.1:8851`，语义推理在 Orin。
- 地面模型 Python、权重路径通过下面的 launch 参数指定。
- 按实机配置 `bounds_min/max`、速度、加速度和飞控外部定位融合。定位输入只保留一个发布源。

PX4 外部视觉融合参数按 [官方外部定位说明](https://docs.px4.io/v1.16/en/ros/external_position_estimation) 配置；本工程输出 FRD 参考的外部位姿，不宣称提供已经验证的外部速度。

## 3. 按顺序启动

**Orin 终端 A：飞控 DDS 链路**（已有 Agent 时复用，不重复启动）：

```bash
MicroXRCEAgent udp4 -p 8888
```

**Orin 终端 B：Odin 独立定位**：

```bash
ros2 launch flyco_real odin.launch.py config:="$PWD/config/vehicle.odin.local.yaml"
```

此入口运行 Odin 原驱动、传感器桥和相机视场节点，可独立使用。数据转换至 `/flyco/odom`、`/flyco/lidar/world`、`/flyco/camera/rgb`，并发送 PX4 外部视觉位姿。位姿转发不依赖相机 TF 是否已经就绪。

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

FlyCo 使用与 PX4 Position 相同的本地位置要求：`mode_req_local_position=false`、`mode_req_local_position_relaxed=true`。PX4 根据 `local_position_invalid_relaxed` 判断该项条件，放宽水平位置误差的有限阈值，仍检查位置有效性与数据新鲜度。角速度、姿态、本地高度以及 FlyCo 自身的定位超时和地面心跳检查继续生效。这个改动只统一位置检查条件，不能消除定位数据中断；编译后重启模式节点即可生效，无需刷写飞控。

工程内 `third_party/px4_msgs` 已直接同步用户提供的 PX4 1.17 定制源码定义；来源提交、文件校验和见该包 `SOURCE.json`。两端在工程根目录执行 `colcon build`，再加载 `install/setup.bash`、重启已有节点即可。模式节点直接使用 `register_ext_component_request_v1`、`register_ext_component_reply_v1`、`arming_check_*_v1` 和 `vehicle_local_position_v1`，无需消息转换节点或额外消息工作区。注册请求明确允许用户在 QGC 中选择 FlyCo。

两端 ROS 域与飞控保持一致；当前实机飞控域为 66。曾经运行过旧转换节点的终端应先结束它；节点终端仅加载本工程环境。若仍显示 `waiting for PX4 registration reader`，检查该终端的域与 `/fmu/in/register_ext_component_request_v1` 的接收端。源码当前为 `feature/neural` 的 `0d092a91…`，与飞控报告的 `feature/vv_agentic` / `f026a1d…` 不同；已核对提供源码的使用接口，实际注册和融合仍需机载验证。

## 验证范围

原 PX4 仿真已验证扫描、QGC 实际界面降落和 RViz 显示。原生 FlyCo 模式已完成正常解锁、Takeoff → FlyCo、预测、完整扫描及落地停桨；ULog 确认全程未进入 Offboard。地面包、Odin 驱动和完整机载算法均在 x86_64 编译通过，新入口完成参数解析。具体证据见 [验收记录](dev-real-status.md)。当前没有连接 Orin/Odin 实机，ARM64 构建、双机网络、设备标定和实飞仍需硬件验收。
