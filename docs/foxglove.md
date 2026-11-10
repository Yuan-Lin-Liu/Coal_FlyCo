# Foxglove 可视化

此入口与原有 RViz2 并列使用。它不启动或修改传感器、分割、预测、规划及飞控功能。现有 RViz2 配置和启动命令继续保留。

## 本机启动

本机已有 Foxglove Desktop 和 ROS Humble `foxglove_bridge`。首次添加本包后，在工程根目录构建：

```bash
cd ~/Coal_FlyCo
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select flyco_foxglove --symlink-install
```

保持原有 Odin/FlyCo 进程运行，另开终端执行：

```bash
cd ~/Coal_FlyCo
FOXGLOVE_DOMAIN=66 bash tools/run_foxglove.sh
```

这会启动独立的转换节点 `rviz_to_foxglove`、官方桥接节点 `flyco_foxglove_bridge`，并打开 Foxglove Desktop，连接 `ws://127.0.0.1:8765`。

首次在 Foxglove 的 **Layouts → Import from file…** 中选择：

```text
~/Coal_FlyCo/runtime/foxglove/flyco_odin.json
```

也提供已生成的布局 `ros2/flyco_foxglove/config/flyco_odin.json`。ROS 节点生成的运行时版本会使用启动时实际保存的 RViz 配置。

## 转换内容

| RViz 内容 | Foxglove 对应内容 |
| --- | --- |
| 世界坐标系、网格、视角 | 3D 面板，默认 `world` |
| PointCloud2 | 原话题及颜色、点大小、可见性 |
| Marker / MarkerArray | 轨迹、预测 Mesh、视野锥等原有 Marker |
| Path | 原有路径话题 |
| Odometry | 独立节点转为 `/foxglove/poses/...` 的 PoseStamped，保留时间和坐标系 |
| 已启用的 Image | 右侧独立图像面板 |
| 分组开关 | 继承到子显示项的可见性 |

Foxglove 直接读取现有点云、图像、路径和 Marker 消息。Odometry 的位姿在这个新增节点内转为专用可视化 PoseStamped，原始消息不变。自定义 `/prediction/predicted_mesh` 用于规划，视觉显示使用现有 `/prediction/mesh_vis`。桥接只开放当前布局涉及的话题和 TF，不开放客户端话题发布或服务调用。

默认保留 RViz 中的显示开关。例如原配置关闭的预测点云仍关闭，可在 Foxglove 的 3D 面板设置中启用 `/mesh_pred/pred_pc`。相机、点云、预测或轨迹只有在原节点输出数据时才会显示。

RViz 插件不能逐像素迁移：Marker 使用消息自带的几何和颜色；原生 RViz 专属按钮、交互工具及未知自定义插件不转换，未知插件会在转换报告中列出。无需把 RViz2 关闭。

## 选项

不打开桌面客户端，只启动桥和转换节点：

```bash
FOXGLOVE_DOMAIN=66 bash tools/run_foxglove.sh desktop:=false
```

指定另一份保存的 RViz 配置或端口：

```bash
FOXGLOVE_DOMAIN=66 bash tools/run_foxglove.sh \
  rviz_config:="$PWD/ros2/flyco_real/config/flyco_real.rviz" port:=8766
```

转换节点监测 RViz 文件的保存变化，并刷新输出 JSON。Foxglove 需要重新导入更新后的 JSON；如果新增话题，也需重启这个独立入口以刷新桥的话题白名单。手动重新导出：

```bash
ros2 service call /rviz_to_foxglove/reload std_srvs/srv/Trigger '{}'
```

仅离线导出，无需运行 ROS 节点：

```bash
ros2 run flyco_foxglove export_foxglove_layout \
  ros2/flyco_real/config/flyco_real.rviz runtime/foxglove/flyco_odin.json
```

## 安装与排查

其他电脑可从 [Foxglove 官方下载页](https://foxglove.dev/download) 获取桌面客户端，通过系统包管理器安装对应的 `ros-humble-foxglove-bridge`。本机下载缓存位于 `.local/deploy/downloads/`。

连接失败先检查桥接启动终端和端口占用；数据缺失先核对 ROS 域是否与现有算法一致。图形桌面不可用时使用 `desktop:=false`，随后从桌面客户端手动建立 Foxglove WebSocket 连接。此入口默认只绑定本机 `127.0.0.1`。

官方参考：[ROS 2 接入](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)、[布局导入](https://docs.foxglove.dev/docs/visualization/layouts)、[命令行连接](https://docs.foxglove.dev/docs/visualization/open-via-cli)。
