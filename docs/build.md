# ROS2 构建与维护

在工程根目录构建，默认 ROS2 Humble 已配置，额外依赖见 [真机部署说明](deploy_orin.md)：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Orin 和地面电脑都构建完整工作区，由 launch 选择运行模块。无需构建脚本，也无需设置 `COLCON_DEFAULTS_FILE`。已有的旧 `runtime/native/*/install` 可以保留，新终端统一加载根目录 `install/setup.bash`。

## 目录与包

| 位置 | 内容 |
|---|---|
| `ros2/flyco_real` | 飞控、传感器适配、任务节点和 launch |
| `third_party/flyco_ros2` | 感知客户端、预测、规划和轨迹算法 |
| `third_party/ros_interfaces` | 4 个自定义消息包 |
| `third_party/px4_msgs` | PX4 DDS 消息 |
| `third_party/drivers/odin_ros_driver` | Odin 驱动与目标架构 SDK |
| `build/` | CMake 缓存与中间产物 |
| `install/` | 节点、库、资源与 ROS2 环境 |
| `log/` | 构建和测试日志 |

根目录 [colcon_defaults.yaml](../colcon_defaults.yaml) 由 `colcon-defaults` 自动读取：只扫描上述源码目录，避免将模型源码、历史运行目录当作 ROS2 包；使用 Ninja，每次最多编译 2 个源文件，限制 Orin 内存占用。默认关闭构建期测试，采用 Release/O1。

机载 CMake 默认构建算法，关闭旧仿真专用控制器。Odin 根据 CMake 的目标架构选择 SDK；ARM 使用本机 Embree 3 与 OMPL，x86 可复用仓库现有的仿真依赖缓存，绝不在 ARM 链接 x86 库。模型环境、权重、CUDA 扩展仍按部署依赖准备，colcon 负责 ROS2 包的构建和安装。

## 日常修改

源码更新后执行 `colcon build`，已有结果会增量复用。首次构建后，只更新入口包也可使用：

```bash
source install/setup.bash
colcon build --packages-select flyco_real
```

查看实际包列表用 `colcon list`，查看依赖关系用 `colcon graph`。构建错误先看 `log/latest_build/<包名>/stdout_stderr.log`；Ninja 的编译器错误可能输出到 stdout，仅看终端的 stderr 摘要会漏掉原因。

PX4 消息定义更新后，仅需构建消息与适配入口；强制重新配置 CMake 以识别新增/移除的消息文件：

```bash
colcon build --packages-select px4_msgs flyco_real --cmake-force-configure
source install/setup.bash
```

节点会依据消息中的 `MESSAGE_VERSION` 选择 DDS 后缀。Agent 连通后直接运行
`ros2 launch flyco_real mode.launch.py`，注册日志与故障排查见
[固件与消息版本](deploy_orin.md#px4-固件与消息版本)。

开发时只对本次改动做必要的本地检查；已有通过结果且未改动的部分复用原记录。
交付流程直接提供编译、启动命令，排查命令在出现问题时按需使用。

如果前 6 个包已完成、`flyco_ros2` 失败，修复后只续编剩余两个包，保留已有缓存：

```bash
colcon build --packages-select flyco_ros2 flyco_real --event-handlers console_direct+
```

确认两个包都显示 `Finished` 后再加载 `install/setup.bash`。安装 Embree 等依赖成功不代表这两个 ROS2 包已经安装完成；构建失败时缺少 `local_setup.bash` 是后续结果。设备信息日志使用 `/proc/cpuinfo`，缺少型号时回退到系统架构，不依赖 x86 的 `cpuid.h`。

原有 `.local/flyco` 仿真构建仍由 `tools/setup_flyco.sh` 显式启用旧控制节点；该兼容入口与根目录标准构建分别维护输出。原生仿真优先加载新的 `install/local_setup.bash`。

## 改造前检查点

- 分支：`dev-real`
- HEAD：`92cb08009b4f6fbc6d42b27e4d1927b71304abd5`
- Git 标签：`checkpoint/before-standard-colcon-20261106`

该标签记录改造前的完整版本。需要对照时可使用 `git show checkpoint/before-standard-colcon-20261106:docs/deploy_orin.md`，无需切换当前工作区。

## 本次验证

x86_64 主机在仅加载 ROS2 基础环境的终端中，从空的根目录 `build/`、`install/` 开始执行 `colcon build`，8 个包全部通过。安装目录的节点、launch、真机参数和核心动态库已检查。本轮只验证构建与安装；Orin ARM64 编译及实机运行仍需在板卡上验收。
