cmake_minimum_required(VERSION 3.16)
project(flyco_ros2 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-Wno-deprecated-declarations -Wno-unused-parameter -fmax-errors=5)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
option(FLYCO_BUILD_LEGACY_SIM_CONTROL "Build Pegasus-specific px4ctrl and joystick nodes" OFF)
set(ROS_DEPS rclcpp ament_index_cpp std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs
  visualization_msgs rosgraph_msgs geographic_msgs tf2 tf2_ros
  tf2_geometry_msgs tf2_sensor_msgs cv_bridge image_transport message_filters
  pcl_conversions quadrotor_msgs perception_msgs airsim_ros_pkgs self_msgs_and_srvs)
if(FLYCO_BUILD_LEGACY_SIM_CONTROL)
  list(APPEND ROS_DEPS mavros_msgs)
endif()
foreach(dep IN LISTS ROS_DEPS)
  find_package(${dep} REQUIRED)
endforeach()
option(FLYCO_BUILD_ALGORITHMS "Build native C++ planning and clustering nodes" ON)
if(FLYCO_BUILD_ALGORITHMS)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search segmentation surface features)
find_package(OpenCV REQUIRED)
# Prefer target-installed dependencies. Existing x86 simulator checkouts can
# reuse their local OMPL cache; never use these x86 libraries on ARM/cross builds.
find_package(ompl QUIET CONFIG)
if(NOT ompl_FOUND)
  set(_sim_ompl "${CMAKE_CURRENT_SOURCE_DIR}/../../.local/flyco/deps/root/usr/share/ompl/cmake")
  if(NOT CMAKE_CROSSCOMPILING AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$"
      AND EXISTS "${_sim_ompl}/omplConfig.cmake")
    find_package(ompl REQUIRED CONFIG PATHS "${_sim_ompl}" NO_DEFAULT_PATH)
    message(STATUS "Using existing simulator OMPL cache: ${ompl_DIR}")
  else()
    find_package(ompl REQUIRED CONFIG)
  endif()
endif()
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
set(P ${CMAKE_CURRENT_SOURCE_DIR}/vendor/planner_simulator/src)
# Never link an x86 prebuilt library on an ARM board.
set(_bundled_default OFF)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(_bundled_default ON)
endif()
option(FLYCO_USE_BUNDLED_EMBREE "Use the legacy x86 Embree library" ${_bundled_default})
if(FLYCO_USE_BUNDLED_EMBREE)
  if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    message(FATAL_ERROR "Bundled Embree is x86-64. Build Embree 3 for the target and set FLYCO_USE_BUNDLED_EMBREE=OFF")
  endif()
  set(EMBREE ${P}/3rdparty/embree-3)
  set(FLYCO_EMBREE_INCLUDE ${EMBREE}/include)
  set(FLYCO_EMBREE_LINK ${EMBREE}/lib/libembree3.so)
else()
  find_package(embree 3 REQUIRED CONFIG)
  set(FLYCO_EMBREE_LINK embree)
  set(FLYCO_EMBREE_INCLUDE ${EMBREE_INCLUDE_DIRS})
endif()
set(FLYCO_COMPONENT_DIRS
  active_perception airsim_simulator/px4ctrl flyco_planner_manager
  hierarchical_coverage_planner path_searching plan_env rgb_point_map
  skeleton_decomp utils/uav_utils viewpoint_manager vis_utils)
include_directories(include ${P} ${P}/3rdparty/libigl/include ${FLYCO_EMBREE_INCLUDE}
  ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
foreach(component IN LISTS FLYCO_COMPONENT_DIRS)
  include_directories(${P}/${component}/include ${P}/${component}/include/px4ctrl)
endforeach()

function(flyco_target name)
  ament_target_dependencies(${name} ${ROS_DEPS})
  target_link_libraries(${name} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OMPL_LIBRARIES}
    ${FLYCO_EMBREE_LINK} yaml-cpp Threads::Threads)
  # libigl is already header-only; its shipped static archive is architecture-specific.
  if(FLYCO_USE_BUNDLED_EMBREE)
    set_property(TARGET ${name} APPEND PROPERTY BUILD_RPATH "${EMBREE}/lib")
    set_property(TARGET ${name} APPEND PROPERTY INSTALL_RPATH "${EMBREE}/lib")
  endif()
  set_property(TARGET ${name} APPEND PROPERTY INSTALL_RPATH "$ORIGIN/..")
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
if(FLYCO_BUILD_LEGACY_SIM_CONTROL)
add_executable(joy_cmd ${P}/airsim_simulator/joy_control/src/joy_cmd.cpp)
flyco_target(joy_cmd)
target_link_libraries(joy_cmd flyco_algorithms)
endif()
add_executable(rgb_point_mapping_node ${P}/rgb_point_map/src/mapping.cpp)
flyco_target(rgb_point_mapping_node)
target_link_libraries(rgb_point_mapping_node flyco_algorithms)

set(SAM ${CMAKE_CURRENT_SOURCE_DIR}/vendor/perception_predictor/src/sam_tracking)
include_directories(${SAM}/include ${SAM}/src/cluster)
add_executable(sam_tracking_node ${SAM}/src/cluster/cluster_node.cpp
  ${SAM}/src/cluster/pc_cluster.cpp ${SAM}/src/cluster/pc_utils.cpp)
flyco_target(sam_tracking_node)
if(FLYCO_BUILD_LEGACY_SIM_CONTROL)
set(CTRL_SOURCES px4ctrl_node.cpp PX4CtrlFSM.cpp PX4CtrlParam.cpp controller.cpp hovthrkf.cpp input.cpp)
list(TRANSFORM CTRL_SOURCES PREPEND "${P}/airsim_simulator/px4ctrl/src/")
add_executable(px4ctrl_node ${CTRL_SOURCES})
flyco_target(px4ctrl_node)
endif()

# The skeleton planner invokes this CPU farthest-point sampler at runtime.
set(FPS ${P}/skeleton_decomp/fps/sampling_cpu/src)
add_executable(kdline ${FPS}/sampler_main.cpp ${FPS}/Interval.cpp
  ${FPS}/KDNode.cpp ${FPS}/KDTreeBase.cpp ${FPS}/Point.cpp ${FPS}/KDLineTree.cpp)
target_compile_definitions(kdline PRIVATE LINE)

# Build the solver from C sources on the target CPU; no checked-in ELF is used.
set(LKH_SOURCE ${P}/hierarchical_coverage_planner/solution/LKH/SRC)
file(GLOB LKH_SOURCES CONFIGURE_DEPENDS ${LKH_SOURCE}/*.c)
add_executable(LKH ${LKH_SOURCES})
target_include_directories(LKH PRIVATE ${LKH_SOURCE}/INCLUDE)
target_compile_definitions(LKH PRIVATE TWO_LEVEL_TREE)
target_compile_options(LKH PRIVATE -fcommon)
target_link_libraries(LKH m)

endif()
set(SAM ${CMAKE_CURRENT_SOURCE_DIR}/vendor/perception_predictor/src/sam_tracking)
ament_python_install_package(flyco_common PACKAGE_DIR python/flyco_common)
ament_python_install_package(sam_tracking PACKAGE_DIR ${SAM}/src/sam_tracking)
set(MESH ${CMAKE_CURRENT_SOURCE_DIR}/vendor/perception_predictor/src/mesh_predict)
ament_python_install_package(mesh_predict PACKAGE_DIR ${MESH}/src/mesh_predict)
ament_python_install_package(pointnet2_ops PACKAGE_DIR ${MESH}/src/pointnet2_ops)
install(DIRECTORY ${MESH}/src/pointnet2_ops/_ext-src
  DESTINATION ${PYTHON_INSTALL_DIR}/pointnet2_ops)
install(PROGRAMS ${SAM}/nodes/tracking_node.py ${SAM}/nodes/prompt_node.py
  ${SAM}/nodes/prompt_ui_cv_node.py ${MESH}/scripts/mesh_predict_node.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${SAM}/config
  DESTINATION share/${PROJECT_NAME}/vendor/perception_predictor/src/sam_tracking)
install(DIRECTORY ${MESH}/config
  DESTINATION share/${PROJECT_NAME}/vendor/perception_predictor/src/mesh_predict)
if(FLYCO_BUILD_ALGORITHMS)
install(TARGETS flyco_algorithms LIBRARY DESTINATION lib)
install(TARGETS flyco_planner traj_server rgb_point_mapping_node sam_tracking_node kdline LKH
  DESTINATION lib/${PROJECT_NAME})
if(FLYCO_BUILD_LEGACY_SIM_CONTROL)
  install(TARGETS joy_cmd px4ctrl_node DESTINATION lib/${PROJECT_NAME})
endif()
endif()
ament_package()
