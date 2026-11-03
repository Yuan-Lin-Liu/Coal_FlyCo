#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Sept. 2024
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the header file of the bridge between mapping and ROS in FlyCo.
 * Copyright    :    Copyright (c) 2026 Chen Feng and Guiyong Zheng.
 * License      :    PolyForm Noncommercial License 1.0.0
 *                   <https://polyformproject.org/licenses/noncommercial/1.0.0/>
 *
 *                   This software is released for noncommercial research and
 *                   educational use only. You may use, modify, and distribute
 *                   this software for noncommercial purposes, subject to the
 *                   terms of the PolyForm Noncommercial License 1.0.0.
 *
 *                   Commercial use, including use in commercial products,
 *                   commercial services, paid consulting, or internal business
 *                   operations, is prohibited without prior written permission
 *                   from the copyright holders.
 *
 *                   This software is provided "as is", without warranty of any
 *                   kind, express or implied.
 * Project      :    FlyCo: Foundation Model-Empowered Drones for Autonomous 3D Structure Scanning in Open-World Environments
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include "plan_env/sdf_map.h"
#include "plan_env/raycast.h"
#include "plan_utils/visibility_st.hpp"
#include "quadrotor_msgs/msg/mesh.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/circular_buffer.hpp>
#include <embree3/rtcore.h>

using std::string;
using std::vector;
using std::list;
using std::shared_ptr;
using std::unique_ptr;

class RayCaster;

namespace flyco {
class SDFMap;

class MapROS {
public:
  MapROS(){
  };
  ~MapROS(){
  };
  void setMap(SDFMap* map);
  void init(const rclcpp::Node::SharedPtr& node);

private:
  void depthPoseQuatCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                             const nav_msgs::msg::Odometry::ConstSharedPtr& pose,
                             const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& quat);
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void cloudPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
                         const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void cloudPoseTrackCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
                              const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void tarEnvPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& target,
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr& env,
                          const nav_msgs::msg::Odometry::ConstSharedPtr& pose);
  void updateESDFCallback();
  void visCallback();
  void partialCallback();
  void partialDataCallback();
  void trackingReinitCallback(const std_msgs::msg::Bool::ConstSharedPtr& bool_msg);
  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy);
  void saveInheritMapCallback();
  // -----------------------
  void publishMapAll();
  void publishMapLocal();
  void publishESDF();
  void publishMappingRange();
  void publishUpdateRange();
  void publishDepth();
  void publishTarget();
  void publishEnvironment();
  // -----------------------
  void proessDepthImage();
  pcl::PointCloud<pcl::PointXYZ> generateAirPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, const Eigen::Matrix3d& R_sensor2world, const Eigen::Vector3d& t_sensor2world);

  SDFMap* map_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry, geometry_msgs::msg::QuaternionStamped>
      SyncPolicyImagePoseQuat;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePoseQuat>> SynchronizerImagePoseQuat;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry> SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>
      SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>
      SyncPolicyTarEnvPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyTarEnvPose>> SynchronizerTarEnvPose;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_visualization_time_{0, 0, RCL_ROS_TIME};
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> pose_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>> quat_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> tracking_cloud_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> tracking_target_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> tracking_env_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> tracking_pose_sub_;
  SynchronizerImagePoseQuat sync_image_pose_quat_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_pose_;
  SynchronizerCloudPose sync_cloud_pose_track_;
  SynchronizerTarEnvPose sync_tar_env_pose_;
  
  // ROS Service
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_local_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_local_inflate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_all_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_all_inflate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_free_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mapping_range_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr update_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr env_pub_;
  rclcpp::TimerBase::SharedPtr esdf_timer_, vis_timer_, partial_timer_, partial_data_timer_, inherit_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr partial_pub_;
  rclcpp::SubscriptionBase::SharedPtr mesh_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tracking_reinit_sub_;

  unique_ptr<rclcpp::WallRate> mapping_rate_ = nullptr;

  rclcpp::TimerBase::SharedPtr update_map_timer_;

  // type of input
  bool depth_pose_input_ = false;
  bool cloud_pose_input_ = false;
  bool depth_gimbal_pose_input_ = false;
  bool cloud_tar_env_pose_input_ = false;

  // ! Depth & Pose & Gimbal
  boost::circular_buffer<sensor_msgs::msg::Image> buffer_depth_dpg_;
  boost::circular_buffer<nav_msgs::msg::Odometry> buffer_pose_dpg_;
  boost::circular_buffer<geometry_msgs::msg::QuaternionStamped> buffer_quat_dpg_;
  void updateMapDepthPoseQuat();

  // ! Depth & Pose
  boost::circular_buffer<sensor_msgs::msg::Image> buffer_depth_dp_;
  boost::circular_buffer<nav_msgs::msg::Odometry> buffer_pose_dp_;
  void updateMapDepthPose();

  // ! Cloud & Pose
  boost::circular_buffer<sensor_msgs::msg::PointCloud2> buffer_cloud_cp_;
  boost::circular_buffer<nav_msgs::msg::Odometry> buffer_pose_cp_;
  void updateMapCloudPose();

  // ! Cloud & Target & Environment & Pose
  boost::circular_buffer<sensor_msgs::msg::PointCloud2> buffer_all_ctep_;
  boost::circular_buffer<nav_msgs::msg::Odometry> buffer_pose_track_;
  boost::circular_buffer<sensor_msgs::msg::PointCloud2> buffer_target_ctep_;
  boost::circular_buffer<sensor_msgs::msg::PointCloud2> buffer_env_ctep_;
  boost::circular_buffer<nav_msgs::msg::Odometry> buffer_pose_ctep_;
  void updateMapCloudTarEnvPose();

  // output
  boost::circular_buffer<pcl::PointCloud<pcl::PointNormal>> buffer_partial_;
  boost::circular_buffer<vector<int>> buffer_partial_vox_ids_;

  // params, depth projection
  Eigen::Matrix3d K_depth_;
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  string frame_id_;
  bool airsim_flag_;
  double mapping_fps_, partial_time_;
  // params, lidar air projection
  double h_fov_, v_fov_min_, v_fov_max_, inf_fov_;
  double h_res_, v_res_;
  double mapping_visible_dist_;
  int dil_pixels_;
  // msg publication
  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_, show_local_map_;
  bool show_depth_, show_local_range_;
  bool show_target_, show_env_;
  bool inflate_vis_;

  // Depth Extrinsic Parameters
  double dep_x_, dep_y_, dep_z_;
  double dep_roll_, dep_pitch_, dep_yaw_;
  Eigen::Vector3d d2body_t_vector_;
  Eigen::Matrix3d d2body_R_matrix_;

  // data
  // flags of map state
  bool local_updated_, esdf_need_update_;
  // input
  Eigen::Vector3d camera_pos_;
  Eigen::Quaterniond camera_q_;
  Eigen::Quaterniond gimbal_q_;
  unique_ptr<cv::Mat> depth_image_;
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  rclcpp::Time map_start_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::QuaternionStamped q_gimbal;

  unique_ptr<RayCaster> raycaster_;

  // inheritance map
  bool en_inherit_, already_inherit_;

  friend SDFMap;
};
}

#endif
