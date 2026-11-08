# ROS2 PX4 Odometry

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS2 Humble](https://img.shields.io/badge/ros2-humble-brightgreen.svg?logo=ros)](https://wiki.ros.org/foxy)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

[中文文档](README_CN.md)

Convert external odometry/pose messages to PX4 local position measurements.

## Node

### `odom_to_px4_node`

- Supported input types: `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseStamped`, `geometry_msgs/msg/PoseWithCovarianceStamped`
- Applies configurable extrinsic translation and rotation
- Converts pose from ENU to NED before sending to PX4

## Run

Use launch file (default config: `nokov_mocap_config.yaml`):

```bash
ros2 launch ros2_px4_odometry generic_odometry.launch.py
```

Use another config file:

```bash
ros2 launch ros2_px4_odometry generic_odometry.launch.py config_file:=fast_lio_config.yaml
```

Run node directly with overrides:

```bash
ros2 run ros2_px4_odometry odom_to_px4_node \
  --ros-args \
  -p message_type:=pose_stamped \
  -p odometry_topic:=/mocap/pose \
  -p extrinsic_translation:="[0.1, 0.0, 0.0]"
```

## Parameters

- `message_type`: `odometry` | `pose_stamped` | `pose_with_covariance_stamped`
- `odometry_topic`: input topic name
- `extrinsic_translation`: `[x, y, z]`
- `extrinsic_rotation_matrix`: 3x3 rotation matrix in row-major 9-element array
- `config_file`: launch argument, YAML file in `config/`

## Dependencies

- ROS2 Humble
- `px4_ros2_cpp`
- `Eigen3`
