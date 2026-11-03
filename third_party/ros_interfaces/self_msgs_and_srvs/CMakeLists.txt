cmake_minimum_required(VERSION 3.16)
project(self_msgs_and_srvs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GlbObsRcv.srv"
  DEPENDENCIES 
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
