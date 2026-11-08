# PX4 / QGroundControl 煤仓仿真

宿主机原生仿真默认使用 **FlyCo 轨迹 → PX4 SITL → Pegasus 电机物理**。RGB-D、雷达、煤堆和规划算法沿用现有工程。QGC 通过独立的 MAVLink 链路连接 PX4；默认 `px4` 路径不用 MAVROS 或 MicroXRCEAgent。`px4-native` 路径使用 DDS，需要 Agent。

## 启动

本机已安装 PX4 SITL 和 QGC，直接运行：

```bash
cd /home/robot/coal_flyco
./launch_warehouse.sh --scene 4 --qgc
# 场景 6：先停止上一个会话，再把 4 换成 6
```

默认 PX4 目录 `~/PX4-Autopilot`，QGC 为 `~/QGroundControl-x86_64.AppImage`。路径不同时：

```bash
./launch_warehouse.sh --scene 4 --qgc \
  --px4-dir /实际目录/PX4-Autopilot \
  --qgc-path /实际目录/QGroundControl.AppImage
```

`--qgc` 新启动的 QGC 使用本次会话的独立英文配置，避免已有版本的中文 `降落` 模式名匹配错误。配置、缓存及地面站日志位于运行目录的 `qgc-*` 子目录；原来的 QGC 用户配置不变。QGC 已运行时复用现有进程；若仍提示未知中文模式，退出已有 QGC，再用 `--qgc` 启动。不用 `--qgc` 也可手动打开 QGC。默认自动连接本机 UDP **14550**；未连接时，在 QGC 的 Application Settings → Comm Links 添加 UDP 链路，监听端口设为 14550。

## 操作

1. 等待 QGC 定位就绪，Isaac 面板点击 **1 Initialize**。此时记录起点并开始发送当前位置保持目标，不解锁。
2. 等待 FlyCo 完成初始化、**2 Initial flight** 按钮可用后再点击。终端状态对应 `ready_for_initial: true`。仿真程序请求 PX4 切入 Offboard、正常解锁，沿 FlyCo 初始化轨迹原地升至 **2 米**并悬停。若 PX4 拒绝，先查看 QGC 的具体原因，再重试；不会强制解锁。
3. FlyCo-UI 发送煤堆文字，等待识别点云和预测网格，依次点击 **3 Enable planning**、**4 Autonomous scan**。初始化轨迹正常结束后保持终点，下一段扫描轨迹到来时自动继续。
4. **Hold** 暂停并保持当前位置；Hold、异常断流或 QGC 接管后，先点 **Resume**，再按需触发 **4 Autonomous scan**。正常终点保持与显式 Hold 分别显示为 `endpoint_wait` 和关闭的 `accept_trajectories`。
5. 点击 Isaac 的 **Land** 或英文 QGC 的 **Land**。PX4 确认 LAND 后停止 Offboard 输出；实际落地、上锁且四电机归零后，状态显示 `px4.landing_complete: true`。

QGC 切换飞行模式后拥有控制权，FlyCo 不会自动切回 Offboard。需要继续任务时显式点击 Resume，再触发规划。这里的自动解锁只属于本机 SITL 仿真，不是 Orin 真机部署入口。

无窗口快速检查：

```bash
./launch_warehouse.sh --scene 4 --headless --no-ui --no-perception --no-rviz
# 另一终端，等待初始化就绪后依次执行
python3 launch.py action initialize
python3 launch.py action initial
python3 launch.py status
python3 launch.py action land
# 确认落地停桨后
python3 launch.py stop
```

`--no-perception` 用于起降通信检查，不包含目标识别和自主扫描。`--headless` 关闭 Isaac 窗口；只有显式传入 `--qgc` 才会启动 QGC，且 QGC 需要可用桌面。

启用感知时，也可以从终端发送目标文字：

```bash
python3 launch.py action text 'A single large pile of black coal.'
```

PX4 会话执行 `stop`、Ctrl+C 或达到 `--duration` 时，会先请求降落并等待停桨，再关闭进程；90 秒内未完成会在日志中报告失败。窗口直接关闭、进程崩溃或强制终止不属于该正常退出流程。

## 集中验收

完成代码改动后运行相关回归测试，再用一个会话验收任务衔接：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider \
  tests/test_px4_backend.py tests/test_qgc_runtime.py
python3 tools/verify_px4.py --scene 4 --qgc
```

脚本启用真实感知和规划，检查起飞终点、预测网格、扫描轨迹接续、持续飞行、Hold/Resume，以及实际落地停桨；每一步写入 `runtime/px4-verification-*/report.json`。默认持续扫描 20 秒仿真时间，可用 `--scan-seconds` 调整。此验收不代表整座煤堆已达到完整覆盖率。

需要验证 QGC 界面降落时使用 `--qgc --qgc-land`；脚本到达降落阶段会输出提示，此时在 QGC 操作 Land。默认路径验证的是 Isaac 后端发送的降落请求。

只补验 QGC 降落可执行 `python3 tools/verify_px4.py --qgc --qgc-land --landing-only`：起飞仅作为准备，跳过感知和扫描。操作前保持桌面解锁；锁屏时无法进行界面确认。

## 首次准备 PX4 / QGC

已有可运行的 PX4 SITL 时不用重新安装。Pegasus 官方测试版本为 PX4 **v1.14.3**；新机器可按其安装说明准备依赖并编译：

```bash
git clone --branch v1.14.3 --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
make px4_sitl_default
```

QGC 从官网下载 Linux AppImage，赋予执行权限 `chmod +x QGroundControl*.AppImage`，将路径传给 `--qgc-path`。Isaac / Pegasus / FlyCo 的原生依赖仍需先准备好；本页不包含首次安装这些环境。

## 排查与实现

- 当前状态：`python3 launch.py status`，查看 `position`、`mode`、`fault` 和 `px4`。
- 轨迹状态：`px4.tracking`、`accept_trajectories`、`endpoint_wait`；降落状态：`landing_state`、`landing_complete`。`received` 和命令 ACK 都不表示已经完成运动。
- PX4 启动或解锁问题：`runtime/latest/px4.log` 及 QGC 消息。
- Isaac / 算法问题：`runtime/latest/isaac.log`、`ros.log`。
- 端口占用：停止其他 SITL；同一项目仅运行一个会话。TCP 4560 用于 Pegasus 传感器/电机；UDP 14540/14580 用于 FlyCo Offboard；UDP 14550/18570 用于 QGC。

PX4 参数、飞行日志存于本次 `runtime/run-*/px4/`；`px4_build.json` 保存实际 PX4 路径及构建版本，便于区分本机自定义固件和官方标签。不会改动已有真机参数或 PX4 源码；默认 PX4 路径设置 `COM_RC_IN_MODE=4`；原生路径设置 `COM_RC_IN_MODE=8` 并优先使用固定中立虚拟摇杆，使 Position 能通过正常解锁检查。上述设置仅作用于本次 SITL；使用模拟 GPS、IMU、气压计和磁力计。FlyCo 仍使用 Isaac 真值定位；PX4 自己运行 EKF。接口转换世界 ENU / PX4 本地 NED，并在初始化时对齐局部原点。物理频率为 250 Hz。

原自定义控制路径仍可用：`./launch_warehouse.sh --flight-controller flyco --scene 4`。Docker 入口默认沿用该路径；本次 PX4/QGC 联动使用宿主机原生环境。

## 原生 FlyCo 模式仿真：ROS2 终端入口

首次使用标准工作区，或拉取代码更新后，在工程根目录编译：

```bash
cd /home/robot/coal_flyco
source /opt/ros/humble/setup.bash
colcon build
```

每个终端先执行：

```bash
cd /home/robot/coal_flyco
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=64 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

仓库已包含本机定制 PX4 对应的消息，无需再加载旧消息覆盖层。先关闭旧 QGC，再启动：

**终端 A：仿真、PX4、QGC、RViz 与传感器桥**：

```bash
ros2 launch flyco_real simulation.launch.py \
  px4_dir:=/home/robot/PX4
```

本机有效 SITL 位于 `/home/robot/PX4`，其他机器将 `px4_dir` 改为实际编译目录。默认 `start_mode:=false start_algorithms:=false`：终端 A 运行仿真、飞控、DDS Agent、传感器桥、地面心跳及 RViz。旧会话应先落地退出，再按此流程启动。

**终端 B：注册、使能及解锁后启动算法**：

```bash
ros2 launch flyco_real flyco.launch.py simulation:=true
```

此入口立即启动 C++ 模式注册和控制桥，在定位与 PX4 就绪后自动使能；等待 FlyCo 模式正常解锁时，规划、感知、聚类、预测和任务节点尚未启动。命令本身不会解锁。

1. 等待 QGC 出现 **FlyCo**，选择该模式并正常 **Arm**。也支持先在 Position 正常解锁，再切入 FlyCo。
2. 入口收到 FlyCo 已解锁的状态后，自动启动算法并初始化规划器，输出 `FlyCo ready`。
3. 地面状态明确、定位和执行器就绪时，自动执行 PX4 Takeoff，升至距本次地面位置约 **2 m** 后返回 FlyCo 悬停。

算法每次 launch 只启动一次；Takeoff 返回 FlyCo 不会重复起飞。初始化期间上锁、切走模式或丢失地面状态会取消待执行的起飞。空中激活可以启动算法，但不自动追加 2 m 起飞。

不要同时运行另一个 `mode.launch.py`。已有独立模式节点时，可用 `flyco.launch.py simulation:=true start_mode:=false` 复用它；`mode.launch.py` 继续保留为单独注册/联调入口。

需要在解锁前使用文字窗口时，可以独立打开：

```bash
ros2 launch flyco_real prompt.launch.py simulation:=true
```

窗口名为 **FlyCo-UI**，点击底部 Text 输入框编辑英文描述，点击 **Send text** 或按 Enter 发送。也可给合并入口增加 `ui:=true`，随算法在解锁后打开。文字发送与自动起飞相互独立；解锁前算法尚未启动时，发送文字应等到算法就绪后再操作。
独立 UI 在仿真模式下从当前会话读取 ROS 域号，避免窗口落入默认域 0 后只有黑屏。
从其他目录启动时，可通过 `project_root:=/实际工程目录` 指定会话所在的工程。

RViz 的 **Camera field of view** 使用 `/flyco/camera/fov`，由相机内参和实际光学坐标 TF 绘制；起飞前和悬停时也能显示红色视锥，无需先产生规划轨迹。已打开的旧 RViz 可直接将该显示项的 Topic 改为此话题。仿真锥顶位于相机光心，即机体前方约 0.125 m；应与机体安装位置对应。更新相机 TF 修复后，先落地停桨，再重启三终端流程。临时独立启动视锥节点：

```bash
ros2 run flyco_real camera_fov.py --ros-args -p simulation:=true -p use_sim_time:=true
```

悬停后，若需要继续扫描，在另一个已加载环境的终端发送文字；等待 RViz 出现目标点云及预测网格，再逐条激活规划和扫描：

```bash
ros2 run flyco_real task.py text 'A single large pile of black coal.' --sim
ros2 run flyco_real task.py enable-planning --sim
ros2 run flyco_real task.py scan --sim
```

需要时执行对应动作：

```bash
ros2 run flyco_real task.py hold --sim
ros2 run flyco_real task.py resume --sim
ros2 run flyco_real task.py land --sim
ros2 run flyco_real task.py status
```

原生仓库仿真将 PX4 本地位置遥测接收超时与机体定位超时分开：PX4 本地遥测允许 1 秒以内延迟，机体定位和指令仍使用各自门限。持续数据失效时停止控制并请求降落；若 PX4 已切 RTL，仿真通过独立 MAVLink 请求就地 Land，避免默认返航先爬升 30 米。诊断保存在控制状态的 `last_control_fault` 和仿真状态的 `last_safety_action` 中。该室内策略仅作用于原生 SITL，真机仍需单独配置和验收 PX4 失效策略。

降落时 QGC 显示 Land。确认实际落地、上锁、停桨后，再结束合并入口 B 和仿真 A；控制状态见 `/flyco/control/status`，任务状态见 `/flyco/mission/status`。原生模式名称以 QGC 和控制节点为准，Isaac 面板可能概括显示 AUTO。

旧 Python 仿真入口保留；`simulation.launch.py start_mode:=true start_algorithms:=true` 可恢复内置控制与算法，此时不要再运行合并入口。

## 本机验收记录（2026-11-06）

- PX4 接口与 QGC 启动隔离回归测试：19 项通过。两个新增用例补齐初始化保持点后，仅重跑这两个失败用例。
- 场景 4 真实会话 `run-20261106-001856-120c2c`：通过规划器就绪门控、Offboard 起飞、2 m 终点保持、文字感知、网格预测、扫描接续和 Hold/Resume；收到 7369 条轨迹指令，飞行位置横跨约 x=-14～17.3 m、y=-6.9～6.4 m。约 77.9 s 仿真时间收到 `flight_end=true`，规划器报告覆盖率 100%。该覆盖率是算法输出，不等同于真实煤堆表面的独立精度评价。
- 同一会话正常退出后，`result.json` 记录 `armed=false`、`landed_state=1`、`landing_complete=true`；PX4 日志记录实际落地及自动上锁。最小降落会话 `run-20261106-003013-09a080` 也记录了相同的完成状态。
- QGC 已实际启动并连接车辆，界面显示英文 Flying/Offboard/Land；2026-11-06 解锁桌面后，真实 Land 按钮及长按空格确认已通过；会话 `run-20261106-101325-67784f` 于约 62.1 s 进入 LAND、68.1 s 完成落地停桨，验收报告 `px4-verification-20261106-101325/report.json` 为成功。RViz 已实际显示点云、FPV 和轨迹，Global Status 为 OK，截图保存在该会话的 `rviz-visible.png`。此前锁屏超时报告保留为失败。
- 最后补充的退出状态快照和验收脚本中断处理只做了静态检查；未重复运行已经通过的飞行流程。

依据：[Pegasus PX4 接口](https://pegasussimulator.github.io/PegasusSimulator/source/features/px4_integration.html)、[Pegasus 安装](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)、[PX4 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard)、[QGC 通信链路](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/comm_links.html)。
