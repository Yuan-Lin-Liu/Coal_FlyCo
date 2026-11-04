# 识别模块对比与语义目标感知接入

2026-11-04，Coal_FlyCo 基线 `840dcc460126`。

接入前的识别算法与之前 FlyCo 原方案一致：EVF-SAM2 文字初始化、SAM2/SAMURAI 后续视频跟踪，开启深度评分与卡尔曼评分（`sam2_kf_score_weight: 0.15`）。外层已经迁移为 ROS2 Humble，传感器和相机位姿来自 Isaac。

逐字节对比了两个项目的原始版本：`PromptSession`、`TrackingPipeline`、`DepthGuidancePolicy`、`sam2_base.py`、`sam2_camera_predictor.py`、`kalman_filter.py` 六个文件完全相同；原生 Provider 的差异主要为 ROS1 日志迁移到 ROS2。对比摘要保存在 `.local/setup/sam-a-integration/original-comparison.json`。

当前默认后端已替换为语义目标感知，使用 GroundingDINO + SAM2.1 Large 视频跟踪，SAM2.1 Tiny 逐帧图像刷新、掩码质量选择及身份恢复。使用的模型运行源代码与先前最终通用版本一致，指纹为 `bf86e33a8f405d511a56ac80e0369995d7eb65a2b68778ed9f32aaac60c19d12`。没有新增煤堆类别、颜色或固定坐标分支。语义目标感知不使用原生 SAMURAI 的卡尔曼模块。

感知代码保存在 `third_party/SAM_AD_Perception/`，模型与上游固定版本依赖由其 `assets.lock.json` 管理并校验 SHA256。本机已准备好 感知模块的全部权重，新增 Large 权重也已加入资产清单。源码不依赖旧 FlyCo 目录；权重、上游依赖和运行记录不会进入 Git。

语义目标感知在独立的 Python 3.12 / Torch 2.7.1 容器运行，ROS2 保持原来的 Python 3.10 / Torch 2.3.1 环境。专用服务为 `coal-flyco-sam-a`，地址 `http://127.0.0.1:8851`；启动器自动准备服务，只回收自己创建的服务，避免与旧 FlyCo 的 8850 服务共享会话。生成的参数快照位于每次会话的 `flyco_params.yaml`。

主要适配包括：根据 ROS2 参数选择 Provider；保留 RGB 哈希、掩码哈希和帧序号核验；根据 ROS2 的 `sec/nanosec` 区分新相机帧；将深度重初始化改为不破坏身份的恢复建议；无可靠掩码时发布重捕获提示；关闭时释放本会话。Isaac 坐标变换、点云投影、聚类与网格接口继续使用原有实现。

原生掩码可能是 0/1，语义目标感知返回 0/255，原 ROS2 发布端直接乘以 255 会造成 uint8 溢出。本次改为按前景归一化透明度。实际观察的 5 帧压缩 RGBA 输出透明度均为 0/255，且每帧原始 RGB 和透明度掩码的 SHA256 均与 语义目标感知服务输出一致。

## 本机验证

- 模型运行库：44 项测试通过。
- 新增 HTTP / 深度恢复接口：7 项测试通过。
- 真实场景 4：初始化、起飞到约 2 m 并悬停、发送原配置的煤堆文字提示、产生掩码及目标点云、网格预测、Hold、降落并停止旋翼，全部通过。
- 观察到目标点云最多 2041 点，首次验收时网格 1846 顶点；这些是链路输出，不是重建精度。
- 无人机正常降落，启动器退出码 0，无节点崩溃，语义目标感知服务随测试正常退出。
- 验证会话：`runtime/run-20261104-153201-752bc5`；汇总：`.local/setup/sam-a-integration/hover-report.json`。
- 掩码预览：`.local/setup/sam-a-integration/coal-live-overlay.png`。

本次仅验证识别模块与下游接口，没有进行自主整圈扫描或像素标注精度评测。接入前已有的 ROS 桥接、控制器、镜像构建修复均予以保留。

## 启动

```bash
cd /home/buaa/Coal_FlyCo
source .local/setup/env.sh
./docker/run.sh --scene 4
```

默认使用语义目标感知。按原流程初始化和起飞，在 FlyCo-UI 选择 **Send text** 发送目标文字；可在 RViz 的 `/tracking/mask/image` 查看分割。语义目标感知只接受文字，原来的点提示需使用原生后端：

```bash
./docker/run.sh --scene 4 --segmentation-backend native
```

新机器首次安装仍用 `docker/setup.sh`，它会额外构建并准备 语义目标感知服务。已有原生工程只补装语义目标感知时使用 `bash tools/setup_semantic_perception.sh`。两个模块的 Python 依赖分开安装，不需要重建 ROS2 C++ 代码，也不需要重启电脑。

手动诊断服务可使用 `python3 tools/semantic_perception_service.py start`、`status`、`stop`；源码更新后需重启该服务，启动时会核对运行源代码指纹。

## 识别效果 Image

RViz 默认启用 `Target perception → Semantic Perception Image`。它显示相机原图、紫红色半透明目标区域和轮廓，订阅 `/perception/segmentation/overlay`。单独查看二值掩码可添加 Image 并选择 `/perception/segmentation/mask`。

图像只使用相同时间戳的相机帧和掩码；失去可靠目标时清除覆盖区域并显示状态。显示独立于点云投影，避免点云为空时看不到二维识别结果。

对外统一使用 `semantic` 后端名称，例如 `--segmentation-backend semantic`；模型服务内部协议、已安装镜像标识及旧入口保留兼容。

## 重捕获状态修复

2026-11-04，实际扫描触发 `reinit_pending` 时，ROS2 发布端曾把无图像的恢复状态当成正常结果读取，报 `KeyError: masked_rgb` 并导致会话关闭。发布端现已明确处理该状态：清空当前掩码、保持节点运行、等待后续恢复。真实 ROS2 回调的两个回归测试覆盖此状态及空预测，均通过。
