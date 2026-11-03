cmake_minimum_required(VERSION 3.16)
project(flyco_ros2 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-Wno-deprecated-declarations -Wno-unused-parameter -fmax-errors=5)
find_package(ament_cmake REQUIRED)
set(ROS_DEPS rclcpp ament_index_cpp std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs
  visualization_msgs rosgraph_msgs mavros_msgs geographic_msgs tf2 tf2_ros
  tf2_geometry_msgs tf2_sensor_msgs cv_bridge image_transport message_filters
  pcl_conversions quadrotor_msgs perception_msgs airsim_ros_pkgs self_msgs_and_srvs)
foreach(dep IN LISTS ROS_DEPS)
  find_package(${dep} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search segmentation surface features)
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
set(P ${CMAKE_CURRENT_SOURCE_DIR}/vendor/planner_simulator/src)
set(EMBREE ${P}/3rdparty/embree-3)
set(FLYCO_COMPONENT_DIRS
  active_perception airsim_simulator/px4ctrl flyco_planner_manager
  hierarchical_coverage_planner path_searching plan_env rgb_point_map
  skeleton_decomp utils/uav_utils viewpoint_manager vis_utils)
include_directories(include ${P} ${P}/3rdparty/libigl/include ${EMBREE}/include
  ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
foreach(component IN LISTS FLYCO_COMPONENT_DIRS)
  include_directories(${P}/${component}/include ${P}/${component}/include/px4ctrl)
endforeach()

function(flyco_target name)
  ament_target_dependencies(${name} ${ROS_DEPS})
  target_link_libraries(${name} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OMPL_LIBRARIES}
    ${P}/3rdparty/libigl/libigl.a ${EMBREE}/lib/libembree3.so yaml-cpp Threads::Threads)
  set_target_properties(${name} PROPERTIES BUILD_RPATH "${EMBREE}/lib" INSTALL_RPATH "${EMBREE}/lib")
endfunction()

set(ALGORITHM_SOURCES
  vis_utils/src/planning_visualization.cpp active_perception/src/perception_utils.cpp
  path_searching/src/astar2.cpp
  plan_env/src/sdf_map.cpp plan_env/src/raycast.cpp plan_env/src/map_ros.cpp
  rgb_point_map/src/rgb_point_map.cpp rgb_point_map/src/rgb_point_map_utils.cpp
  viewpoint_manager/src/viewpoint_manager.cpp
  skeleton_decomp/src/sk_decomp.cpp skeleton_decomp/src/Extra_Del.cpp skeleton_decomp/src/adaptive_utils.cpp
  hierarchical_coverage_planner/src/fcplanner_pp.cpp hierarchical_coverage_planner/src/local_replan.cpp
  hierarchical_coverage_planner/src/hcsolver.cpp hierarchical_coverage_planner/src/hctraj.cpp
  hierarchical_coverage_planner/src/planner_node.cpp hierarchical_coverage_planner/src/manual_node.cpp
  hierarchical_coverage_planner/src/init_traj.cpp)
list(TRANSFORM ALGORITHM_SOURCES PREPEND "${P}/")
add_library(flyco_algorithms SHARED ${ALGORITHM_SOURCES})
flyco_target(flyco_algorithms)

add_executable(flyco_planner ${P}/flyco_planner_manager/src/flyco_planner_node.cpp
  ${P}/flyco_planner_manager/src/flyco_planner_manager.cpp)
flyco_target(flyco_planner)
target_link_libraries(flyco_planner flyco_algorithms)
add_executable(traj_server ${P}/flyco_planner_manager/src/traj_server.cpp)
flyco_target(traj_server)
target_link_libraries(traj_server flyco_algorithms)
add_executable(joy_cmd ${P}/airsim_simulator/joy_control/src/joy_cmd.cpp)
flyco_target(joy_cmd)
target_link_libraries(joy_cmd flyco_algorithms)
add_executable(rgb_point_mapping_node ${P}/rgb_point_map/src/mapping.cpp)
flyco_target(rgb_point_mapping_node)
target_link_libraries(rgb_point_mapping_node flyco_algorithms)

set(SAM ${CMAKE_CURRENT_SOURCE_DIR}/vendor/perception_predictor/src/sam_tracking)
include_directories(${SAM}/include ${SAM}/src/cluster)
add_executable(sam_tracking_node ${SAM}/src/cluster/cluster_node.cpp
  ${SAM}/src/cluster/pc_cluster.cpp ${SAM}/src/cluster/pc_utils.cpp)
flyco_target(sam_tracking_node)
set(CTRL_SOURCES px4ctrl_node.cpp PX4CtrlFSM.cpp PX4CtrlParam.cpp controller.cpp hovthrkf.cpp input.cpp)
list(TRANSFORM CTRL_SOURCES PREPEND "${P}/airsim_simulator/px4ctrl/src/")
add_executable(px4ctrl_node ${CTRL_SOURCES})
flyco_target(px4ctrl_node)

# The skeleton planner invokes this CPU farthest-point sampler at runtime.
set(FPS ${P}/skeleton_decomp/fps/sampling_cpu/src)
add_executable(kdline ${FPS}/sampler_main.cpp ${FPS}/Interval.cpp
  ${FPS}/KDNode.cpp ${FPS}/KDTreeBase.cpp ${FPS}/Point.cpp ${FPS}/KDLineTree.cpp)
target_compile_definitions(kdline PRIVATE LINE)

# Python entry points and assets are registered by tools/install_flyco_runtime.py.
install(TARGETS flyco_algorithms LIBRARY DESTINATION lib)
install(TARGETS flyco_planner traj_server joy_cmd px4ctrl_node
  rgb_point_mapping_node sam_tracking_node kdline DESTINATION lib/${PROJECT_NAME})
ament_package()
