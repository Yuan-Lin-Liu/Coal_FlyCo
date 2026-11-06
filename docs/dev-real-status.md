# dev-real 验收记录（2026-11-06）

仿真基线提交为 `2726c4f`，在其上创建本地分支 `dev-real`。本记录对应本机测试，运行产物保存在 `runtime/`，不随 Git 发布。

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

本机未连接 Orin/Odin/PX4 真机，尚无 ARM64 编译、设备在线标定、双机无线网络负载和硬件实飞证据。ARM SDK 与目标架构构建路径已交付，以上项目留待硬件验收。`dev-real` 为本地分支，尚未推送远端。

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
