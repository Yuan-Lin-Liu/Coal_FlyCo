# Coal_FlyCo：Isaac Sim + FlyCo 煤仓无人机仿真

Isaac Sim 5.1 / Pegasus 提供无人机物理、RGB-D 相机和激光雷达；FlyCo 负责目标分割、网格预测与覆盖轨迹规划。宿主机原生模式默认由 **PX4 SITL** 执行飞控控制，支持 **QGroundControl** 联动；FlyCo 使用 Isaac 真值定位，PX4 使用模拟传感器运行 EKF。

当前默认使用通用 **语义目标感知（Semantic Perception）**，原 EVF-SAM2/SAMURAI 方案可用 `--segmentation-backend native` 对照运行。接入差异、验证结果和模型准备方式见 [识别模块说明](docs/semantic-perception.md)。

## 新机器安装（Docker）

源码、两个煤堆 STL 和两个 USD 场景保存在此仓库；Isaac、ROS、Python 环境和模型权重由安装脚本获取，不进入 Git。发布者已确认取得当前 FlyCo ROS2 修改版的公开发布授权；第三方来源和原始条款见 [THIRD_PARTY.md](THIRD_PARTY.md)。

宿主机需要 Linux x86_64、Python 3.10+、兼容的 NVIDIA RTX 显卡及驱动、Docker Engine、Docker Buildx 插件（`docker buildx version` 可用）、NVIDIA Container Toolkit、`python3-yaml`、`curl`。已验证配置包括 RTX 3090 / 驱动 580.178.04，以及 RTX 4090 D / 驱动 580.196.07（见 [本机复现记录](REPRODUCTION.md)）。首次安装包含数十 GB 的下载与构建，请预留至少 100 GB；缓存、权重、构建结果保留在本地。

```bash
git clone https://github.com/Yuan-Lin-Liu/Coal_FlyCo.git
cd Coal_FlyCo
# 如缺少宿主机 Python YAML 包：sudo apt install python3-yaml
# 阅读 NVIDIA Isaac Sim EULA 后设置 ACCEPT_EULA=Y。
ACCEPT_EULA=Y ./docker/setup.sh
ACCEPT_EULA=Y ./docker/run.sh --scene 4
# 切换场景：先停止当前会话，再启动
ACCEPT_EULA=Y ./docker/run.sh --scene 6
```

`docker/setup.sh` 基于 NVIDIA Isaac Sim 和 OSRF ROS2 Humble Desktop 官方镜像，构建三个本地镜像：Isaac Sim 5.1 + Pegasus、ROS2 Humble + FlyCo，以及独立的 语义目标感知服务。运行时通过宿主机网络上的 TCP 端口通信，共享本次会话目录；源码以只读方式挂载，权重以只读方式挂载，日志和缓存单独持久化。宿主机不需要安装 ROS 或完整 CUDA Toolkit。

默认显示 Isaac、FlyCo-UI、RViz，需可用的本地 X11/XWayland 显示和 Xauthority。使用 `--headless` 运行无窗口仿真。原有终端管理命令继续可用，例如 `python3 launch.py status`、`python3 launch.py action land`、`python3 launch.py stop`。结束管理器会停止其创建的两个容器。Docker 默认保留原自定义控制路径；PX4/QGC 联动使用下文宿主机原生入口。

```bash
./docker/setup.sh --check          # 检查镜像、权重及固定版本缓存
ACCEPT_EULA=Y ./docker/setup.sh --rebuild # 修改源码后重建；保留下载的模型
ACCEPT_EULA=Y ./docker/run.sh --scene 6 --headless
```

ROS 软件源访问受限时，可设置 `COAL_ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu`，仍使用 ROS 原始签名校验。资源下载可中断重试，已通过校验的文件会复用。Google Drive 限流时，可把原始 `checkpoint.pt` 放入 `.local/flyco/models/mesh/` 后重试，或用 `MESH_CHECKPOINT_URL` 指定同一文件的镜像地址。文件仍必须匹配 `config/models.json` 的 SHA256。Hugging Face 模型固定到该清单中的提交版本。镜像构建会根据本机 GPU 编译 CUDA 扩展；可通过 `TORCH_CUDA_ARCH_LIST` 覆盖目标架构，当前 CUDA 11.8 / Torch 2.3.1 组合已在 RTX 3090（Ampere）和 RTX 4090 D（Ada）上验证。

NVIDIA 官方容器和安装要求：[Isaac Sim 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html)、[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)。

若下载依赖本机 HTTP 代理，应同时配置宿主机的 `http_proxy` / `https_proxy`、Docker daemon 的镜像下载代理，以及 Docker 客户端配置中的构建代理。构建期间访问 `127.0.0.1` 代理时，可设置 `COAL_BUILD_NETWORK=host`。此选项只影响镜像构建网络；不要通过关闭 TLS 校验解决证书错误。

`dev-real` 日常仿真先运行 `ros2 launch flyco_real simulation.launch.py`，再运行 `ros2 launch flyco_real flyco.launch.py simulation:=true`，同一入口完成模式注册与使能，在 FlyCo 正常解锁后启动算法，初始化后自动升至约 2 m。其他任务使用 `ros2 run flyco_real task.py ...`；逐条指令见 [ROS2 仿真入口](docs/px4_qgc.md#原生-flyco-模式仿真ros2-终端入口)。

`dev-real` 负责 Odin + Orin 原生 ROS2 部署及 PX4 动态 FlyCo 模式，见 [部署说明](docs/deploy_orin.md)。论文要求地面运行预测，Orin 运行定位、感知、规划与轨迹执行；原生 FlyCo 模式已通过起飞、预测、扫描和降落闭环，详见 [验收记录](docs/dev-real-status.md)；Orin/Odin 实机验收待硬件连接后进行。

## dev-real 的标准 ROS2 构建

Orin 和地面电脑在工程根目录使用相同命令；依赖清单见 [真机部署说明](docs/deploy_orin.md)。

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

输出为 `build/`、`install/`、`log/`，根目录 `colcon_defaults.yaml` 自动提供默认配置，无需导出 `COLCON_DEFAULTS_FILE`。设备运行分工由 ROS2 launch 选择。[构建与维护说明](docs/build.md) 包含包结构、单包更新及本次改造前的 Git 检查点。

## 本机原生启动与场景选择

PX4 / QGC 的准备、操作和排查见 [联动快速说明](docs/px4_qgc.md)。本机已有 PX4 SITL 和 QGC 时：

```bash
# 二选一；不加参数默认使用场景 4
./launch_warehouse.sh --scene 4 --qgc
./launch_warehouse.sh --scene 6 --qgc
```

| 场景 | 煤堆模型 | 尺寸（米） | USD |
|---|---|---|---|
| `4` | `assets/coal_piles/coal_pile_final.stl` | 12×6×5 | `coal_warehouse.usda` |
| `6` | `assets/coal_piles/coal_pile_solid.stl` | 约 10.61×5.34×5 | `scenes/coal_warehouse_6.usda` |

每个场景只有一个煤堆，底面中心为 `(8, 0, 0)`。无人机出生在 `(-14, 0, 0.15)` 并朝向 `+X`，所以煤堆相对原位置沿无人机视线向后移了 8 米；两个 STL 均等比例放大 2 倍，显示、碰撞与雷达采用同一模型。`--pile` 与 `--scene` 等效；切换前先结束当前会话。

默认打开 Isaac、FlyCo-UI 和 RViz2。可使用 `--no-rviz` 关闭自动 RViz，或使用 `--headless` 无窗口运行。`ISAACSIM_PATH` 可指定其他 Isaac 安装。

## 操作

1. 等待算法启动，点 **1 Initialize**，等待起点初始化完成。
2. 点 **2 Initial flight**，仿真请求 PX4 切入 Offboard、正常解锁，原地垂直起飞至 2 m 后悬停。
3. 在 **FlyCo-UI** 点选煤堆并发送提示；默认语义目标感知后端请点击 **send text** 发送文字，点提示需使用 `--segmentation-backend native`。
4. 在 RViz 的 **Semantic Perception Image** 查看紫红色识别覆盖图。等待目标点云和预测网格出现，再点 **3 Enable planning**。
5. 点 **4 Autonomous scan** 开始扫描。
6. 点 Isaac 的 **Land** 或 QGC 的 **降落**，确认落地、`PX4_*_DISARMED` 且旋翼停止（旧控制路径显示 `GROUNDED`）。

初始化轨迹正常结束后，PX4 保持终点并自动接续下一段扫描轨迹。**Hold** 保持当前位置；主动暂停、异常断流或 QGC 接管后，用 **Resume** 恢复 FlyCo 控制。面板的 `RECEIVED` 表示收到操作，实际执行情况以模式、位置和运动为准。PX4 降落完成以 `px4.landing_complete: true` 为准；正常退出会先等待降落停桨。

```bash
python3 launch.py status         # 当前会话和飞行状态
python3 launch.py stop           # 结束当前会话
python3 launch.py logs           # 关键日志，重复信息只显示一次
python3 launch.py logs --verbose # 完整输出；Ctrl+C 只退出查看
python3 launch.py rviz           # 单独打开当前会话的 RViz
python3 launch.py start --scene 6 # 后台启动
python3 launch.py action land    # 终端执行面板操作
python3 launch.py action text 'A single large pile of black coal.' # 终端发送目标文字
```

`action` 还支持 `initialize`、`initial`、`plan`、`scan`、`hold`、`resume`、`disarm`。默认 ROS 域为 64、TCP 端口为 8870，可通过 `--domain`、`--port` 修改。同一项目同时运行一个会话。

## 配置

| 文件 | 用途 |
|---|---|
| `config/coal_warehouse.yaml` | 公共传感器、出生点、煤堆尺寸、总览视角及 UI 默认文字 |
| `config/scenes.yaml` | 场景 4/6 的 USD、模型和配置覆盖项 |
| `config/initial_waypts.yaml` | 初始化航点，格式 `[x, y, z, 云台俯仰角, 机体航向角]` |
| `config/coal_warehouse.rviz` | RViz 图层和视角 |
| `third_party/flyco_ros2/config/flyco.yaml` | FlyCo 算法参数 |

位置单位为米，世界 Z 轴向上，航点角度为度。出生点当前为 `(-14, 0, 0.15)`，朝向 +X。修改起飞高度时只需改航点第三个数值，保持 X/Y 与出生点一致。

UI 默认文字通过以下参数预填，仍需点击 **send text** 发送：

```yaml
initial_text_prompt: "A single large, irregular pile of black coal on the warehouse floor."
```

配置在启动时读取；运行目录内保存本次实际使用的配置与航点快照。修改模型、缩放、煤堆位置或起降台位置后，重新生成对应场景，再重启仿真：

```bash
./isaac_python.sh tools/rebuild_scene.py --scene 4
./isaac_python.sh tools/rebuild_scene.py --scene 6
# 同时生成两个场景使用 --scene all
```

## 源码与依赖

| 位置 | 内容 |
|---|---|
| `sim/` | Isaac、Pegasus、传感器和旋翼物理适配 |
| `bridge/` | ROS 接口、FlyCo 启动与终端日志 |
| `third_party/flyco_ros2/` | 当前使用的 FlyCo 感知、规划、控制源码 |
| `third_party/ros_interfaces/` | 自定义 ROS 消息与服务 |
| `third_party/EVF-SAM/` | 原生后端的文字目标初始化模型代码 |
| `third_party/SAM_AD_Perception/` | 默认语义目标感知模块及模型资产清单 |
| `.local/flyco/` | Python 环境、CUDA/OMPL、权重、模型缓存及构建结果 |
| `.local/isaacsim/`、`.local/PegasusSimulator/` | 本机仿真环境 |

原生模式不依赖其他 FlyCo 工程目录，但需要已有本地 Isaac / Pegasus、ROS2 Humble、Python 环境、CUDA/OMPL 和模型权重。`tools/setup_flyco.sh` 用于重新编译已有环境；新机器请优先使用上面的 Docker 安装入口。大型环境和权重不随 Git 上传。原生环境入口为 `tools/flyco_env.sh`：

```bash
./tools/setup_flyco.sh
```

Pegasus 5.1 的本地适配补丁位于 `patches/`，使用 `tools/apply_pegasus_patch.sh` 应用。第三方源码的许可证与版权声明保留；FlyCo 的使用限制见 `LICENSE.FlyCo`。
