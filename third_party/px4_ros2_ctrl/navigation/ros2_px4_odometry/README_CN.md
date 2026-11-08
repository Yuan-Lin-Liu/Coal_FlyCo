# ROS2 PX4 Odometry

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS2 Humble](https://img.shields.io/badge/ros2-humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

[English document](README.md)

将外部里程计/位姿消息转换为 PX4 本地位置测量输入。

## 节点

### `odom_to_px4_node`

- 支持输入类型：`nav_msgs/msg/Odometry`、`geometry_msgs/msg/PoseStamped`、`geometry_msgs/msg/PoseWithCovarianceStamped`
- 支持可配置外参平移与旋转
- 发送到 PX4 前自动执行 ENU 到 NED 转换

## 运行

使用 launch 文件（默认配置：`nokov_mocap_config.yaml`）：

```bash
ros2 launch ros2_px4_odometry generic_odometry.launch.py
```

使用其他配置文件：

```bash
ros2 launch ros2_px4_odometry generic_odometry.launch.py config_file:=fast_lio_config.yaml
```

直接启动节点并覆盖参数：

```bash
ros2 run ros2_px4_odometry odom_to_px4_node \
  --ros-args \
  -p message_type:=pose_stamped \
  -p odometry_topic:=/mocap/pose \
  -p extrinsic_translation:="[0.1, 0.0, 0.0]"
```

## 参数

- `message_type`: `odometry` | `pose_stamped` | `pose_with_covariance_stamped`
- `odometry_topic`: 输入话题名
- `extrinsic_translation`: `[x, y, z]`
- `extrinsic_rotation_matrix`: 3x3 旋转矩阵，按行优先展开为 9 元素数组
- `config_file`: launch 参数，位于 `config/` 下的 YAML 文件

## 依赖项

- ROS2 Humble
- `px4_ros2_cpp`
- `Eigen3`
