cmake_minimum_required(VERSION 3.5)
project(ros2_px4_odometry)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
set(BUILD_SHARED_LIBS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})

######### odom_to_px4_node #########
add_executable(odom_to_px4_node src/odom_to_px4_node.cpp)
ament_target_dependencies(odom_to_px4_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros Eigen3 px4_ros2_cpp)

install(
        TARGETS odom_to_px4_node
        DESTINATION lib/${PROJECT_NAME}
)

######### install #########
install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
