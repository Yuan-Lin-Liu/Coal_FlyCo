# 本机配置与复现记录

日期：2026-11-04（Asia/Shanghai）。源码基线：`840dcc4`，另有本次安装修复，见 Git diff。

**当前状态：环境检查、场景 4 完整流程与正常退出验证通过。验证会话已停止，可手动启动。场景 6 的后续验证已按用户要求停止。**

| 项目 | 本机配置 |
|---|---|
| 宿主机 | Ubuntu 22.04.4 LTS，x86_64 |
| GPU | NVIDIA GeForce RTX 4090 D，24,564 MiB |
| NVIDIA 驱动 | 580.196.07 |
| Docker Engine | 29.1.3 |
| Buildx | v0.29.1，安装于工程 `.local/docker-cli/cli-plugins/` |
| 仿真 | Isaac Sim 5.1.0 + Pegasus v5.1.0 + 仓库适配补丁 |
| 算法环境 | ROS2 Humble、Python 3.10、Torch 2.3.1 / CUDA 11.8 |
| CUDA 编译架构 | `8.9`，与本机显卡匹配 |

已通过 CUDA 11.8 容器实测 GPU 透传。用户已在本次会话明确接受 NVIDIA EULA。

在本机终端加载项目配置后使用 Docker 入口：

```bash
cd /home/buaa/Coal_FlyCo
source .local/setup/env.sh
# 完整构建完成后检查：
./docker/setup.sh --check
# 前台启动，包含 Isaac、FlyCo-UI 和 RViz：
./docker/run.sh --scene 4
# 场景切换前先结束当前会话：
python3 launch.py stop
./docker/run.sh --scene 6
```

`.local/setup/env.sh` 保存本机代理 `127.0.0.1:7897`、本地 Docker 客户端目录、EULA 接受状态及 GPU 架构，不进入 Git。运行前需确保本机代理仍可用。Docker daemon 使用的是机器原有的代理配置，本次未修改系统 Docker 配置。

本次配置使用 Docker；`launch_warehouse.sh` 和 `isaac_python.sh` 的宿主机原生安装入口不等同于 Docker 入口。

| 已验证项目 | 场景 4 | 场景 6 |
|---|---|---|
| Isaac/Pegasus 真实物理与场景加载 | 通过 | 通过 |
| 仿真时长 / 物理步数 | 3 秒 / 360 | 3 秒 / 360 |
| RGB / 深度 / 激光雷达帧数 | 各 30 帧 | 各 30 帧 |
| RGB 分辨率 | 640 × 480 | 640 × 480 |
| 雷达单帧最大点数 | 28,036 | 28,073 |
| 程序正常退出，USD 未改动 | 通过 | 通过 |
| FlyCo 闭环飞行与正常退出 | 通过 | 后续验证按要求停止 |

上表的传感器专项检查期间无人机保持地面状态。随后场景 4 使用真实 FlyCo 节点完成了闭环验证：

| 场景 4 闭环项目 | 实测结果 |
|---|---|
| 起飞与悬停 | 起飞检查高度 1.986 m；发送目标提示前稳定在 2.000 m |
| 目标初始化 | 使用 `config/coal_warehouse.yaml` 的默认文字提示，分割与目标点云通过 |
| 网格预测 | 首次验证网格 1,492 顶点 |
| 自主扫描 | 实际产生位移；额外观察 20 秒仿真，FlyCo 报告覆盖 68.79% |
| 降落 | `GROUNDED`，四个旋翼指令全部为 0 |
| 降落时覆盖报告 | 78.17% |
| 总仿真时长 / 控制消息 / RGB 帧 | 61.39 秒 / 6,760 / 611 |
| 退出 | 管理器退出码 0，控制器正常退出，无节点崩溃、无未回收回调异常 |
| 场景文件 | USD 校验不变 |

扫描验证采用有限时长运行；覆盖数值来自 FlyCo 对当前模型的估计。本次没有进行全覆盖完成率或重建精度的定量验收。

最新完整报告：`.local/setup/scene-4-flight-report.json`。对应原始会话：`runtime/run-20261104-141116-cd6c90/`，包含 `ros.log`、`isaac.log`、`trajectory.jsonl`、`camera.png` 和 `result.json`。

原始报告、相机图像和仿真日志：

- `runtime/sensor-check-scene-4/sensor-check.json`、`camera.png`、`result.json`。
- `runtime/sensor-check-scene-6/sensor-check.json`、`camera.png`、`result.json`。
- `.local/setup/container-gpu-check.txt`：GPU 容器透传结果。
- `.local/setup/models-check.log`：仓库模型清单检查结果。
- `.local/setup/flyco-build.log`、`.local/setup/isaac-build.log`：构建日志。

模型存放在 `.local/flyco/models/`。四项独立权重按 `config/models.json` 的 SHA256 校验；三个 Hugging Face 模型固定到清单提交，并核对权重的官方摘要。下载缓存和 OCI 镜像归档保留在 `.local/setup/`，方便中断后复用。

本次修复了 Buildx 缺失时的安装诊断，增加可选的宿主机构建网络，并使已准备好的模型缓存可跳过重复下载。NKSR 使用 `pyproject.toml` 声明包名和依赖，基础镜像自带的 setuptools 59 无法正确读取；在 NKSR 构建前固定安装 setuptools 69.5.1。上游声明见 [NKSR 固定版本的打包配置](https://github.com/nv-tlabs/NKSR/blob/e40336845e67761343a756788e5a98b827d4a143/package/pyproject.toml)。

本机为加速首次下载，使用 `.local/setup/Dockerfile.flyco.cached` 构建 FlyCo；它将官方 wheel 下载为本地缓存并保留官方 SHA256 检查，依赖版本与仓库配方相同。常规安装入口仍为 `docker/setup.sh`，机器本地的缓存构建入口为 `.local/setup/build_flyco_cached.sh`。

退出阶段还修复了控制器提前销毁 ROS 上下文、桥接层排队回调以及分割线程未收尾的问题；场景 4 的复测同时检查了实际降落和正常退出。
