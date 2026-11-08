# dev-real 验收记录（2026-11-06）

仿真基线提交为 `2726c4f`，在其上创建本地分支 `dev-real`。本记录对应本机测试，运行产物保存在 `runtime/`，不随 Git 发布。

## 2026-11-07：PX4 v1.17 自定义消息同步

- 从用户指定的本机 `/home/robot/PX4-Autopilot` 同步 238 个消息与 1 个服务定义，
  并逐文件核对一致；来源为 `feature/neural` 的 `0d092a91f0c0417ab888b4bc37331484e6d46b9d`。
  注册请求/回复、解锁检查请求/回复和本地位置现使用 `_v1`，模式明确保持可由用户选择。
- x86_64 本机只续编 `px4_msgs` 和 `flyco_real`，两包完成，实际编译耗时约 6 分 8 秒。
- 版本 1 注册与解锁回复已有本地通过记录，保留这一项回归检查；报告见
  `runtime/px4-v117-migration/pytest-initial.xml` 中对应的成功用例。
- 按交付要求移除新增的独立 PX4 预检入口及其 6 项模拟测试。常规操作直接运行
  `mode.launch.py`，出现问题再使用已有 `task.py status` 查看注册状态；不重复已通过的检查。
- 精简后仅增量编译入口包 `flyco_real`（约 1 秒），并确认已安装的
  `mode.launch.py --show-args` 可正常解析；注册用例复用上面的通过记录。
- 此次未重跑飞行、扫描、模型推理或规划器。用户飞控报告的 `feature/vv_agentic` /
  `f026a1d46ea94233da7b6d8ba4675c72a8cbc300` 与指定源码提交不同，实机注册以终端
  `Registered FlyCo` 和 QGC 显示为准。`ref_alt=NaN` 的原生起飞限制保留。

## 已通过

| 项目 | 结果与证据 |
|---|---|
| 原 PX4 仿真功能 | 文字感知、预测、轨迹接续、扫描、Hold/Resume；`run-20261106-001856-120c2c` 收到 `flight_end=true`，算法覆盖率 100% |
| QGC 界面降落 | 真实 Land 按钮及长按空格确认；`px4-verification-20261106-101325/report.json` 成功，实际落地、上锁、停桨 |
| RViz | `run-20261106-101325-67784f/rviz-visible.png`：点云、FPV、轨迹及 Global Status OK |
| FlyCo 动态模式 | PX4 注册模式、健康检查和模式执行器；QGC 实际显示 FlyCo，截图 `run-20261106-105129-c6d167/qgc-flyco-mode.png` |
| 原生模式完整任务 | `run-20261106-120035-5d472d/native-mode-verification.json` 全部阶段成功：注册 → 正常解锁 → Takeoff → FlyCo → 感知/预测 → 扫描结束 → Land → 上锁停桨 |
| 原生模式真实性 | 同一会话 `native-flight-audit.json` 根据 PX4 ULog 核对：FlyCo nav_state 23、Takeoff 17、Land 18；全程未进入 Offboard 14，MAVLink Offboard 目标发送数为 0 |
| 原生扫描结果 | 算法覆盖率约 99.58%，最高高度 2.074 m；最终高度约 0.062 m、`armed=false`、四个电机输出为 0 |
| 构建 | 地面 7 个包、Odin 驱动及完整机载 C++ 算法在 x86_64 主机编译成功；结果分别位于 `runtime/native/ground`、`odin`、`onboard` |
| 启动与回归 | Odin/native/ground/semantic 入口完成 ROS2 参数解析；原仿真 19 项、原生包 27 项不同用例有通过记录，修复后只重跑受影响的失败用例 |

覆盖率是规划器输出，不是煤堆表面的独立精度测量。旧失败记录保留，不能用其中的注册成功代替飞行成功。

## 关键实现

- 原生模式通过 PX4 external-mode API 注册，并声明位置/姿态控制要求，不发布 OffboardControlMode。
- 起飞由模式执行器调度 PX4 内置 Takeoff，完成后返回 FlyCo，并保持请求的世界坐标高度；原生入口关闭旧初始化起飞轨迹，避免重复执行。
- 本机固件起飞后的小幅 heading-only reset 已按新鲜世界位姿重新对齐；位置重置、大幅航向跳变、定位失效仍停止控制。ULog 确认当时仅航向计数器改变，历史 `delta_z` 值不代表起飞后的高度重置。
- Odin 独立定位入口提供标定、传感器适配和外部视觉位姿；控制入口不自动解锁，暂停和异常恢复使用显式服务。
- 仿真原生入口支持匹配固件的 `px4_msgs` 覆盖层。本机测试固件为定制 `0d092a91f0…`，不能据此宣称所有官方固件版本均已验证。

## 交付范围

按论文 §7.1.1，地面运行预测，Orin 运行其余算法；地面 launch 默认包含 RViz，语义服务在 Orin。运行顺序和依赖见 [部署说明](deploy_orin.md)。

本机未连接 Orin/Odin/PX4 真机，尚无 ARM64 编译、设备在线标定、双机无线网络负载和硬件实飞证据。ARM SDK 与目标架构构建路径已交付，以上项目留待硬件验收。分支分工为 `main` 仿真、`dev-real` 原生模式与真机部署。

## ROS2 终端入口更新

- `simulation.launch.py` 启动仿真环境，默认不内置控制节点；`mode.launch.py simulation:=true` 在另一个终端启动常驻模式节点，自动注册 FlyCo。
- `task.py` 新增 enable/disable、select-mode、arm、takeoff、hold/resume、land、status；全部任务可使用 `ros2 run`。`--sim` 自动适配 Isaac 定位和仿真时间。
- 只更新构建 `flyco_real`，4 项新增 ROS2 任务检查通过，涵盖服务拒绝反馈、无定位时仍能请求降落、动态模式 ID、正常解锁消息与状态确认、仿真 Joy 释放及文字发送。修复测试数组比较后只重跑该失败用例。
- 新入口完成命令发现、参数解析、临时空管理器启动退出及独立模式节点启动退出检查。模式配置会提取节点参数，避免将混合设备配置直接作为 ROS 参数文件。
- 本次没有重跑完整扫描；原闭环飞行证据仍见上表。入口检查日志位于 `runtime/ros2-entrypoint-check/`。

## Position 切入后的自动起飞

- 终端 A 运行仿真、传感器桥与 RViz；B 通过 `mode.launch.py` 注册并在就绪后自动使能；C 通过 `flyco.launch.py` 运行算法、自动初始化和激活任务。
- 按用户确认的操作：在地面以 Position 正常解锁，再切入 FlyCo；自动执行一次约 2 m 起飞并悬停。未解锁、空中切模式、节点启动时已经处于 FlyCo、执行器未取得控制权等条件均不会自行起飞。
- 本轮 10 项新增检查通过，覆盖上述触发条件、故障后使能、显式禁用、正常输入源和仿真输入失联。
- 真实 SITL 会话 `run-20261106-134009-29dec4` 的 `automatic-takeoff-verification.json` 为 SUCCESS。`automatic-takeoff-audit.json` 核对 ULog：Position 2 → FlyCo 23 → Takeoff 17 → FlyCo 23 → Land 18；全程没有 Offboard，只有一次 Takeoff 和一次自动起飞请求。
- 最高世界高度 2.051 m（地面约 0.062 m），最终落地上锁、四电机输出为零；悬停后未重复爬升。没有重复完整扫描。
- 原生 SITL 提供明确优先级的中立虚拟摇杆，满足 Position 正常解锁条件，避免已打开 QGC 的虚拟摇杆与仿真输入交替触发接管；只修改会话内 PX4 参数，真机仍使用真实遥控器。两次修正前的失败记录保留在先前会话中。

## 相机视锥和文字窗口

- 原红色视锥 `/traj_server/fov_vis` 由轨迹服务器收到轨迹后才发布；PX4 原生起飞不产生该轨迹，因此出现悬停时没有视锥的问题。
- 新增独立 `camera_fov.py`，通过实际 CameraInfo 和最新光学坐标 TF 发布 `/flyco/camera/fov`，不依赖规划轨迹；TF 失效后删除标记。仿真传感器入口和真机传感器入口自动启动，RViz 配置同步更新。
- 当前会话 `run-20261106-142129-c50a47` 只读验证得到 14 条红色视锥消息，光学坐标系正确，包含 16 个线段端点；结果见 `camera-fov-verification.json`。没有重跑飞行测试或重启飞行控制节点。
- `ros2 launch flyco_real prompt.launch.py simulation:=true` 可独立打开 FlyCo-UI，当前桌面已确认窗口显示；英文描述通过底部 Text / Send text 或 Enter 发送。命令行 `task.py text ... --sim` 继续可用。

## 相机视锥位置对齐修复

- 实测发现仿真相机 TF 的世界位置误以 `base_link` 为父坐标发布，导致机体位置被重复叠加，视锥明显落后于实际机体。
- 根因是 IMU 和里程计共用同一个 Header；修改 IMU 的 frame_id 连带修改了后续相机 TF 的父坐标。现改为独立 IMU Header，并为相机世界位姿明确填写 `world` 父坐标。
- 两个位置/朝向的回归检查通过，验证相机中心只相对机体偏移实际安装距离，并验证前向光线只进行一次世界变换。没有重复飞行测试。
- 仿真安装偏移为机体前方 0.125 m；修复生效后锥顶对应相机光心，保留这一真实偏移。运行中的旧桥接进程需在落地后重启三终端流程才会使用修复。

## 扫描中突然爬升的诊断与修复

- 用户会话 `run-20261106-151518-15a5ce` 在约 94 s（PX4 日志时间）由 FlyCo 切到 RTL；最高世界高度 30.017 m，随后返航落地上锁。ULog 只有一条 Takeoff 命令，排除重复 2 m 起飞。
- 控制日志首个故障为 PX4 本地位置 DDS 接收超过 0.3 s；桥接停止输出，外部模式健康检查变为失败，PX4 按其固件模式回退策略进入 RTL。PX4 日志明确记录 `30 m above destination`，原始证据保存为该会话的 `unexpected-climb-incident.json`。
- 本次独立 `px4_local_timeout` 默认仍为 0.3 s；仿真 launch 设置为 1 s，区分 DDS 调度延迟与机体里程计失效。`last_control_fault` 保留故障原因和三个输入的接收年龄。
- 原生仓库 SITL 持续失效时请求 Land；独立 MAVLink 后端将 RTL 转为就地 Land，避免室内返航爬升。该策略仅作用于原生 SITL，真机默认 PX4 失效策略不受此改动。
- 4 项定向回归检查通过，覆盖短暂停顿、持续断流降落、真机默认行为和旧后端隔离。实际断流注入验收结果见后续记录。

- 实际断流注入验收 `run-20261106-162019-35fd28/feedback-loss-verification.json` 为 SUCCESS：正常悬停后，0.45 s PX4 本地位置接收停顿被容忍；随后持续停顿超过 1 s，在约 1.017 s 时请求 Land，实际落地上锁停桨。最高世界高度 2.031 m，ULog 确认未进入 RTL。
- 当前用户会话已按授权正常落地并结束；定向验收会话也已正常退出。首次验收脚本过早请求解锁被 PX4 拒绝，保留为失败记录；修正为 Position 正常解锁后通过。本轮没有重复完整扫描。

## 标准 colcon 工作区

- 改造前 HEAD 为 `92cb08009b4f6fbc6d42b27e4d1927b71304abd5`，Git 标签 `checkpoint/before-standard-colcon-20261106` 保存该版本。
- 工程根目录新增自动读取的 `colcon_defaults.yaml`，移除两套需要手动指定的构建配置。两端统一 `colcon build` 和 `source install/setup.bash`，输出为根目录的 `build/`、`install/`、`log/`。
- 在未加载旧接口或算法覆盖层、未设置 `COLCON_DEFAULTS_FILE` 的新终端中，只 source ROS2 后执行一次全量 `colcon build`：8 个包全部成功，耗时约 13 分 25 秒。源码生成的 Ninja 编译池限制同时编译 2 个源文件。
- 新安装目录验证通过：8 个包索引、7 个原生可执行程序及依赖库、10 个 launch 入口和真机参数资源。核心算法库、规划器、轨迹服务器的动态符号检查通过；任务 CLI 和模式 launch 参数可正常解析。
- 编译器输出包含现有 ROS2/PCL 与新版 CMake 的策略提示，以及 x86 Embree/TBB 搜索路径提示；已确认动态库与符号完整。本轮没有重复飞行测试，也没有重新编译已通过的模块。
- 此次实际构建验证为 x86_64，ARM64 仍需在 Orin 上安装目标架构依赖并执行硬件验收。详细维护方法见 [构建说明](build.md)。验证产物为 `runtime/standard-workspace-verification.json`。
