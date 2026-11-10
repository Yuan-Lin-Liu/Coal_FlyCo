# Orin + Odin + PX4 部署

分支 `dev-real`，ROS2 Humble。

论文 §7.1.1 的分工：**地面电脑运行表面预测；Orin 运行定位、感知、规划和轨迹执行**。

## 1. 依赖与编译

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

不需要重编已通过的 `flyco_ros2` 算法。所有运行终端重新加载更新后的环境，避免继续导入
旧工作区的 `px4_msgs`。保持与飞控一致的 ROS 域；例如已使用域 66 的现场终端继续使用 66。

启动注册与使能节点：

```bash
ros2 launch flyco_real mode.launch.py
```

## 2. 每个终端的环境与设备配置

Orin 和地面电脑每个终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

ROS2 域、网络范围和 RMW 实现暂不指定，真机部署时根据地面电脑、Orin 和现有 PX4 DDS 配置统一确认。若现有通信已正常，直接复用；需要调整时再确定 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY`、`RMW_IMPLEMENTATION` 及飞控的 `UXRCE_DDS_DOM_ID`。部署时还需保证两端时间同步和 DDS 网络互通。仿真继续按 [仿真说明](px4_qgc.md) 使用域 64。

其余 `config/vehicle.odin.local.yaml` 配置在真机部署时确认：

- Orin 的 `segmentation_endpoint` 保持 `http://127.0.0.1:8851`，语义推理在 Orin。
- 地面模型 Python、权重路径通过下面的 launch 参数指定。
- 按实机配置 `bounds_min/max`、速度、加速度和飞控外部定位融合。定位输入只保留一个发布源。

PX4 外部视觉融合参数按 [官方外部定位说明](https://docs.px4.io/v1.16/en/ros/external_position_estimation) 配置；本工程输出 FRD 参考的外部位姿，不宣称提供已经验证的外部速度。

## 3. 按顺序启动

**Orin 终端 B：Odin**：

```bash
ros2 launch flyco_real odin.launch.py config:="$PWD/config/vehicle.odin.local.yaml"
```

此入口运行 Odin 原驱动、传感器桥和相机视场节点，可独立使用。数据转换至 `/flyco/odom`、`/flyco/lidar/world`、`/flyco/camera/rgb`，并发送 PX4 外部视觉位姿。

**Orin 终端 C：本机语义推理**：

```bash
ros2 launch flyco_real semantic.launch.py python:=/实际Orin模型环境/bin/python
```
如启用语义 API token，在 Orin 的服务与算法终端设置相同 `SAM_AD_API_TOKEN`。

**地面终端 A：预测、心跳与 RViz2**：

```bash
ros2 launch flyco_real ground.launch.py config:="$PWD/config/vehicle.odin.local.yaml" \
  model_python:=/实际地面模型环境/bin/python models_dir:=/实际权重目录 rviz:=true
```

**地面终端 B：独立图传与文字输入 UI**：

```bash
ros2 launch flyco_real prompt.launch.py
```

UI 使用地面电脑的 ROS Python/OpenCV 环境，不需要预测或分割模型环境；通过 `/flyco/camera/rgb` 显示机载图像，通过 `/query_text` 发送目标描述。两端沿用相同 ROS 域和可互通的网络配置；需要显式指定时，给 UI 增加 `domain:=实际域号`。`ground.launch.py` 和机载 `flyco.launch.py` 均不再启动 UI。

**Orin 终端 D：模式注册与解锁后启动算法**：

```bash
ros2 launch flyco_real flyco.launch.py config:="$PWD/config/vehicle.odin.local.yaml"
```

## 4. 飞行操作

确认定位和地面预测链路正常，等待 RC/QGC 出现 **FlyCo**，选择后在地面正常解锁；也支持在 Position 解锁后切入 FlyCo。解锁后算法自动启动，初始化完成输出 `FlyCo ready`，任务节点调度 PX4 内置 Takeoff，升至距本次地面位置约 2 m 后返回 FlyCo 悬停。返回 FlyCo 不会重复起飞；未解锁、空中或落地状态未知时不自动起飞。

地面终端 B 的 UI 可在解锁前独立打开。起飞悬停后，在 **FlyCo-UI** 底部 Text 输入英文描述，点击 **Send text** 或按 Enter 发送。

任务节点收到文字后等待新目标点云及预测网格，自动开启规划、跟踪和扫描；完成后悬停。也可通过地面终端发送文字：

```bash
ros2 run flyco_real task.py text '一个煤堆'
```

## 5. 地面 RViz2 与核心排查

地面 `ground.launch.py rviz:=true` 自动载入真机配置，固定坐标系为 `world`，观察机体姿态、点云、图像、预测和轨迹。红色相机视锥由 `/flyco/camera/fov` 根据实际标定和光学坐标 TF 显示，独立于是否已生成轨迹。手动打开时：

两端 ROS 域与飞控保持一致；当前实机飞控域为 66。