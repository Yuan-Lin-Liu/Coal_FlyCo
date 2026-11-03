cmake_minimum_required(VERSION 3.16)
project(airsim_ros_pkgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Altimeter.msg"
  "msg/CarControls.msg"
  "msg/CarState.msg"
  "msg/Environment.msg"
  "msg/GPSYaw.msg"
  "msg/GimbalAngleEulerCmd.msg"
  "msg/GimbalAngleQuatCmd.msg"
  "msg/VelCmd.msg"
  "msg/VelCmdGroup.msg"
  "srv/Land.srv"
  "srv/LandGroup.srv"
  "srv/Reset.srv"
  "srv/SetGPSPosition.srv"
  "srv/SetLocalPosition.srv"
  "srv/Takeoff.srv"
  "srv/TakeoffGroup.srv"
  DEPENDENCIES geographic_msgs geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
