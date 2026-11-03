#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <flyco_ros2/parameters.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
 * Description  :    This file is the main file of the bridge between mapping and ROS in FlyCo.
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

#include "plan_env/map_ros.h"

namespace flyco {

void MapROS::setMap(SDFMap* map) 
{
  this->map_ = map;

  return;
}

void MapROS::init(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  last_visualization_time_ = node_->now();
  fx_ = flyco::parameter_or(node_, "map_ros.fx", -1.0);
  fy_ = flyco::parameter_or(node_, "map_ros.fy", -1.0);
  cx_ = flyco::parameter_or(node_, "map_ros.cx", -1.0);
  cy_ = flyco::parameter_or(node_, "map_ros.cy", -1.0);
  depth_filter_maxdist_ = flyco::parameter_or(node_, "map_ros.depth_filter_maxdist", -1.0);
  depth_filter_mindist_ = flyco::parameter_or(node_, "map_ros.depth_filter_mindist", -1.0);
  depth_filter_margin_ = flyco::parameter_or(node_, "map_ros.depth_filter_margin", -1);
  k_depth_scaling_factor_ = flyco::parameter_or(node_, "map_ros.k_depth_scaling_factor", -1.0);
  skip_pixel_ = flyco::parameter_or(node_, "map_ros.skip_pixel", -1);

  esdf_slice_height_ = flyco::parameter_or(node_, "map_ros.esdf_slice_height", -0.1);
  visualization_truncate_height_ = flyco::parameter_or(node_, "map_ros.visualization_truncate_height", -0.1);
  visualization_truncate_low_ = flyco::parameter_or(node_, "map_ros.visualization_truncate_low", -0.1);
  show_occ_time_ = flyco::parameter_or(node_, "map_ros.show_occ_time", false);
  show_esdf_time_ = flyco::parameter_or(node_, "map_ros.show_esdf_time", false);
  show_all_map_ = flyco::parameter_or(node_, "map_ros.show_all_map", false);
  show_local_map_ = flyco::parameter_or(node_, "map_ros.show_local_map", false);
  show_depth_ = flyco::parameter_or(node_, "map_ros.show_depth", false);
  show_local_range_ = flyco::parameter_or(node_, "map_ros.show_local_range", false);
  show_target_ = flyco::parameter_or(node_, "map_ros.show_target", false);
  show_env_ = flyco::parameter_or(node_, "map_ros.show_env", false);
  inflate_vis_ = flyco::parameter_or(node_, "map_ros.show_inflate", false);
  frame_id_ = flyco::parameter_or(node_, "map_ros.frame_id", string("world"));
  this->airsim_flag_ = flyco::parameter_or(node_, "map_ros.airsim_flag", true);
  mapping_fps_ = flyco::parameter_or(node_, "map_ros.mapping_fps", -1.0);
  partial_time_ = flyco::parameter_or(node_, "map_ros.partial_time", -1.0);

  // * Type of Sensors Input
  depth_pose_input_ = flyco::parameter_or(node_, "map_ros.dp", false);
  cloud_pose_input_ = flyco::parameter_or(node_, "map_ros.cp", false);
  depth_gimbal_pose_input_ = flyco::parameter_or(node_, "map_ros.dgp", false);
  cloud_tar_env_pose_input_ = flyco::parameter_or(node_, "map_ros.ctep", false);

  // * Depth Extrinsic Parameters
  dep_x_ = flyco::parameter_or(node_, "map_ros.extrinsic_x", 0.0);
  dep_y_ = flyco::parameter_or(node_, "map_ros.extrinsic_y", 0.0);
  dep_z_ = flyco::parameter_or(node_, "map_ros.extrinsic_z", 0.0);
  dep_roll_ = flyco::parameter_or(node_, "map_ros.extrinsic_roll", 0.0);
  dep_pitch_ = flyco::parameter_or(node_, "map_ros.extrinsic_pitch", 0.0);
  dep_yaw_ = flyco::parameter_or(node_, "map_ros.extrinsic_yaw", 0.0);
  d2body_t_vector_ << dep_x_, dep_y_, dep_z_;
  d2body_R_matrix_ = Eigen::AngleAxisd(dep_yaw_, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(dep_pitch_, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(dep_roll_, Eigen::Vector3d::UnitX());
  
  // * LiDAR Air Projection
  h_fov_ = flyco::parameter_or(node_, "map_ros.h_fov", 360.0);
  v_fov_min_ = flyco::parameter_or(node_, "map_ros.v_fov_min", 0.0);
  v_fov_max_ = flyco::parameter_or(node_, "map_ros.v_fov_max", 0.0);
  inf_fov_ = flyco::parameter_or(node_, "map_ros.inf_fov", 0.0);
  h_res_ = flyco::parameter_or(node_, "map_ros.h_res", 0.2);
  v_res_ = flyco::parameter_or(node_, "map_ros.v_res", 0.2);
  mapping_visible_dist_ = flyco::parameter_or(node_, "sdf_map.max_ray_length", 0.0);
  dil_pixels_ = flyco::parameter_or(node_, "map_ros.dil_pixels", 0);

  // * K matrix
  K_depth_.setZero();
  K_depth_(0, 0) = fx_; //fx
  K_depth_(1, 1) = fy_; //fy
  K_depth_(0, 2) = cx_; //cx
  K_depth_(1, 2) = cy_; //cy
  K_depth_(2, 2) = 1.0;

  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  int num = 25;
  double mapping_time = 1.0/this->mapping_fps_;
  double partial_data_time = 2.0*mapping_time < this->partial_time_? 2.0*mapping_time : this->partial_time_;

  // * Output
  this->buffer_partial_ = boost::circular_buffer<pcl::PointCloud<pcl::PointNormal>>(num);
  this->buffer_partial_vox_ids_ = boost::circular_buffer<vector<int>>(num);

  // * Timer
  this->esdf_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(0.05), std::bind(&MapROS::updateESDFCallback, this));
  this->vis_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(mapping_time), std::bind(&MapROS::visCallback, this));
  this->partial_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(this->partial_time_), std::bind(&MapROS::partialCallback, this));
  this->partial_data_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(partial_data_time), std::bind(&MapROS::partialDataCallback, this));

  // * Publisher
  this->map_all_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_all", rclcpp::QoS(1).reliable());
  this->map_all_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_all_inflate", rclcpp::QoS(1).reliable());
  this->map_local_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local", rclcpp::QoS(1).reliable());
  this->map_local_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/occupancy_local_inflate", rclcpp::QoS(1).reliable());
  this->map_unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/unknown", rclcpp::QoS(1).reliable());
  this->map_free_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/free", rclcpp::QoS(1).reliable());
  this->esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/esdf", rclcpp::QoS(1).reliable());
  this->mapping_range_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/sdf_map/mapping_range", rclcpp::QoS(1).reliable());
  this->update_range_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/sdf_map/update_range", rclcpp::QoS(1).reliable());
  this->depth_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/depth_cloud", rclcpp::QoS(1).reliable());
  this->target_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/target", rclcpp::QoS(1).reliable());
  this->env_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/sdf_map/env", rclcpp::QoS(1).reliable());
  this->partial_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/map_ros/partial_observation", rclcpp::QoS(1).reliable());

  // * Subscriber
  this->depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(node_, "/map_ros/depth", rclcpp::QoS(25).reliable().get_rmw_qos_profile()));
  this->cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(node_, "/map_ros/cloud", rclcpp::QoS(25).reliable().get_rmw_qos_profile()));
  this->pose_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(node_, "/map_ros/pose", rclcpp::QoS(25).reliable().get_rmw_qos_profile()));
  this->quat_sub_.reset(new message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>(node_, "/gimbal_pose", rclcpp::QoS(25).reliable().get_rmw_qos_profile()));
  this->tracking_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(node_, "/map_ros/tracking_all_cloud", rclcpp::QoS(25).reliable().get_rmw_qos_profile()));
  this->tracking_target_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(node_, "/map_ros/tracking_target_cloud", rclcpp::QoS(25).reliable().get_rmw_qos_profile()));
  this->tracking_env_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(node_, "/map_ros/tracking_env_cloud", rclcpp::QoS(25).reliable().get_rmw_qos_profile()));
  this->tracking_pose_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(node_, "/map_ros/tracking_pose", rclcpp::QoS(25).reliable().get_rmw_qos_profile()));
  // this->tracking_reinit_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/tracking/tracking_reinit", rclcpp::QoS(1).reliable(), std::bind(&MapROS::trackingReinitCallback, this, std::placeholders::_1));
  this->joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::QoS(50).reliable(), std::bind(&MapROS::joyCallback, this, std::placeholders::_1));

  // * Timer
  this->inherit_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&MapROS::saveInheritMapCallback, this));

  if (this->depth_gimbal_pose_input_ == true)
  {
    // * Buffer
    this->buffer_depth_dpg_ = boost::circular_buffer<sensor_msgs::msg::Image>(num);
    this->buffer_pose_dpg_ = boost::circular_buffer<nav_msgs::msg::Odometry>(num);
    this->buffer_quat_dpg_ = boost::circular_buffer<geometry_msgs::msg::QuaternionStamped>(num);
    // * Data Interface
    sync_image_pose_quat_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePoseQuat>(MapROS::SyncPolicyImagePoseQuat(25), *depth_sub_, *pose_sub_, *quat_sub_));
    sync_image_pose_quat_->registerCallback(std::bind(&MapROS::depthPoseQuatCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    // * Operational Function
    this->update_map_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(mapping_time), std::bind(&MapROS::updateMapDepthPoseQuat, this));
  }
  
  if (this->depth_pose_input_ == true)
  {
    // * Buffer
    this->buffer_depth_dp_ = boost::circular_buffer<sensor_msgs::msg::Image>(num);
    this->buffer_pose_dp_ = boost::circular_buffer<nav_msgs::msg::Odometry>(num);
    // * Data Interface
    sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(MapROS::SyncPolicyImagePose(25), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(std::bind(&MapROS::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
    // * Operational Function
    this->update_map_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(mapping_time), std::bind(&MapROS::updateMapDepthPose, this));
  }
  
  if (this->cloud_pose_input_ == true)
  {
    // * Buffer
    this->buffer_cloud_cp_ = boost::circular_buffer<sensor_msgs::msg::PointCloud2>(num);
    this->buffer_pose_cp_ = boost::circular_buffer<nav_msgs::msg::Odometry>(num);
    // * Data Interface
    sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(MapROS::SyncPolicyCloudPose(25), *cloud_sub_, *pose_sub_));
    sync_cloud_pose_->registerCallback(std::bind(&MapROS::cloudPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
    // * Operational Function
    this->update_map_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(mapping_time), std::bind(&MapROS::updateMapCloudPose, this));
  }

  if (this->cloud_tar_env_pose_input_ == true)
  {
    // * Buffer
    this->buffer_all_ctep_ = boost::circular_buffer<sensor_msgs::msg::PointCloud2>(num);
    this->buffer_pose_track_ = boost::circular_buffer<nav_msgs::msg::Odometry>(num);
    this->buffer_target_ctep_ = boost::circular_buffer<sensor_msgs::msg::PointCloud2>(num);
    this->buffer_env_ctep_ = boost::circular_buffer<sensor_msgs::msg::PointCloud2>(num);
    this->buffer_pose_ctep_ = boost::circular_buffer<nav_msgs::msg::Odometry>(num);
    // * Data Interface
    sync_cloud_pose_track_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(MapROS::SyncPolicyCloudPose(25), *cloud_sub_, *pose_sub_));
    sync_cloud_pose_track_->registerCallback(std::bind(&MapROS::cloudPoseTrackCallback, this, std::placeholders::_1, std::placeholders::_2));
    sync_tar_env_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyTarEnvPose>(MapROS::SyncPolicyTarEnvPose(25), *tracking_target_sub_, *tracking_env_sub_, *tracking_pose_sub_));
    sync_tar_env_pose_->registerCallback(std::bind(&MapROS::tarEnvPoseCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    // * Operational Function
    this->update_map_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(mapping_time), std::bind(&MapROS::updateMapCloudTarEnvPose, this));
  }

  map_start_time_ = node_->now();
  this->mapping_rate_ =  make_unique<rclcpp::WallRate>(this->mapping_fps_);

  double resolution_ = map_->getResolution();
  Eigen::Vector3d origin, size;
  map_->getRegion(origin, size);
  raycaster_.reset(new RayCaster);
  raycaster_->setParams(resolution_, origin);

  en_inherit_ = false;
  already_inherit_ = false;

  return;
}

void MapROS::visCallback() 
{
  if (show_all_map_) 
  {
  // Limit the frequency of all map
    static double tpass = 0.0;
    const auto current_time = node_->now();
    tpass += (current_time - last_visualization_time_).seconds();
    last_visualization_time_ = current_time;
    if (tpass > 0.1) 
    {
      publishMapAll();
      tpass = 0.0;
    }
  }

  if (this->show_local_map_)
  {
    publishMapLocal();
    publishESDF();
  }
  if (this->show_depth_)
    publishDepth();
  if (this->show_local_range_)
  {
    publishMappingRange();
    publishUpdateRange();
  }
  if (this->show_target_)
    publishTarget();
  if (this->show_env_)
    publishEnvironment();

  return;
}

void MapROS::updateESDFCallback() 
{
  if (!esdf_need_update_) return;
  auto t1 = node_->now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = node_->now();
  esdf_time_ += (t2 - t1).seconds();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).seconds());
  esdf_num_++;
  if (show_esdf_time_)
    RCLCPP_WARN(rclcpp::get_logger("flyco.plan_env"), "ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).seconds(), esdf_time_ / esdf_num_, max_esdf_time_);
  
  return;
}

void MapROS::partialCallback()
{
  if (this->buffer_partial_.empty()) return;

  pcl::PointCloud<pcl::PointNormal> cloud1 = this->buffer_partial_.back();
  vector<int> vox_ids = this->buffer_partial_vox_ids_.back();
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  for (auto id : vox_ids)
  {
    map_->md_->target_publish_buffer_[id] = true;
  }
  this->partial_pub_->publish(cloud_msg);

  // cout << "partial size : " << cloud1.points.size() << endl;

  this->buffer_partial_.clear();
  this->buffer_partial_vox_ids_.clear();

  return;
}

void MapROS::partialDataCallback()
{
  pcl::PointNormal pt;
  pcl::PointCloud<pcl::PointNormal> cloud1;
  vector<int> vox_ids;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) 
      {
        // target
        int vox_id = map_->toAddress(x, y, z);
        if (map_->md_->target_buffer_[vox_id] == 1)
        {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          if (!map_->md_->target_publish_buffer_[vox_id])
          {
            cloud1.push_back(pt);
            vox_ids.push_back(vox_id);
          }
        }
      }
  
  if ((int)cloud1.points.size() == 0) return;

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;

  this->buffer_partial_.push_back(cloud1);
  this->buffer_partial_vox_ids_.push_back(vox_ids);

  return;
}

void MapROS::trackingReinitCallback(const std_msgs::msg::Bool::ConstSharedPtr& bool_msg)
{
  if (bool_msg->data == true)
  {
    RCLCPP_ERROR(rclcpp::get_logger("flyco.plan_env"), "Tracking Reinit!");
    for (int i=0; i<(int)map_->md_->target_buffer_.size(); ++i)
    {
      map_->md_->target_buffer_[i] = 0;
      map_->md_->target_publish_buffer_[i] = false;
      map_->md_->env_buffer_[i] = 0;
    }

    int num = 25;
    this->buffer_partial_ = boost::circular_buffer<pcl::PointCloud<pcl::PointNormal>>(num);
    this->buffer_partial_vox_ids_ = boost::circular_buffer<vector<int>>(num);
  }

  return;
}

void MapROS::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy)
{
  sensor_msgs::msg::Joy joy_msg;
  if (Joy) joy_msg = *Joy;

  // Button 'down_direction' to save current map for inheritance flight
  if (joy_msg.axes[7] < 0.0) this->en_inherit_ = true;

  return;
}

void MapROS::saveInheritMapCallback()
{
  if (!this->en_inherit_ || this->already_inherit_) return;

  this->already_inherit_ = true;

  string map_path = (ament_index_cpp::get_package_share_directory("flyco_ros2") + "/vendor/planner_simulator/src/plan_env") + "/inherit_map";
  string occ_file = map_path + "/occupied.pcd";
  string free_file = map_path + "/free.pcd";
  string offset_file = map_path + "/offset.yaml";

  pcl::PointCloud<pcl::PointXYZ>::Ptr occ_pcd(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_pcd(new pcl::PointCloud<pcl::PointXYZ>);

  map_->getOccPcd(occ_pcd);
  map_->getFreePcd(free_pcd);

  cout << "Occupied Points: " << occ_pcd->points.size() << endl;
  cout << "Free Points: " << free_pcd->points.size() << endl;

  if ((int)occ_pcd->points.size() == 0 || (int)free_pcd->points.size() == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("flyco.plan_env"), "No Points in the Map!");
    return;
  }

  occ_pcd->width = occ_pcd->size();
  occ_pcd->height = 1;
  occ_pcd->is_dense = true;

  free_pcd->width = free_pcd->size();
  free_pcd->height = 1;
  free_pcd->is_dense = true;

  pcl::io::savePCDFileBinaryCompressed(occ_file, *occ_pcd);
  pcl::io::savePCDFileBinaryCompressed(free_file, *free_pcd);

  double map_x_offset, map_y_offset;
  map_->getXYoffset(map_x_offset, map_y_offset);
  cout << "Map Offset: " << map_x_offset << ", " << map_y_offset << endl;

  YAML::Node node;
  node["map_x_offset"] = map_x_offset;
  node["map_y_offset"] = map_y_offset;

  ofstream fout(offset_file, ios::trunc);
  fout << node;
  fout.close();

  return;
}

/**
 * @brief Callback function for depth, pose, and gimbal messages.
 *
 * This function is called when new depth, pose, and quaternion messages are received.
 * It processes the received data and updates the map accordingly.
 *
 * @param img  The depth image message in [sensor] frame.
 * @param pose The body pose message in [world] frame.
 * @param quat The gimbal quaternion message in [gimbal] frame.
 */
void MapROS::depthPoseQuatCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                                   const nav_msgs::msg::Odometry::ConstSharedPtr& pose,
                                   const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& quat) 
{
  // ! AirSim Simulation
  if (pose->child_frame_id == "X" || pose->child_frame_id == "O") return;
  // ! ---------------------

  sensor_msgs::msg::Image cur_img = *img;
  nav_msgs::msg::Odometry cur_pose = *pose;
  geometry_msgs::msg::QuaternionStamped cur_quat = *quat;
  this->buffer_depth_dpg_.push_back(cur_img);
  this->buffer_pose_dpg_.push_back(cur_pose);
  this->buffer_quat_dpg_.push_back(cur_quat);

  return;
}

void MapROS::updateMapDepthPoseQuat()
{
  if (this->buffer_depth_dpg_.empty() || this->buffer_pose_dpg_.empty() || this->buffer_quat_dpg_.empty()) return;

  sensor_msgs::msg::Image img = this->buffer_depth_dpg_.back();
  nav_msgs::msg::Odometry pose = this->buffer_pose_dpg_.back();
  geometry_msgs::msg::QuaternionStamped quat = this->buffer_quat_dpg_.back();

  tf2::Quaternion tfQuat;
  tf2::fromMsg(quat.quaternion, tfQuat);
  tf2::Matrix3x3 m(tfQuat);
  double roll_gimbal, pitch_gimbal, yaw_gimbal;
  m.getRPY(roll_gimbal, pitch_gimbal, yaw_gimbal);

  Eigen::Vector3d body2world_pos_, gimbal2body_pos_;
  Eigen::Matrix3d body2world_R_matrix, gimbal2body_R_matrix;
  Eigen::Quaterniond body2world_R_quat, gimbal2body_R_quat;

  body2world_pos_(0) = pose.pose.pose.position.x; // in world coordinate
  body2world_pos_(1) = pose.pose.pose.position.y; // in world coordinate
  body2world_pos_(2) = pose.pose.pose.position.z; // in world coordinate
  gimbal2body_pos_ = this->d2body_t_vector_;      // hyper param in body
                              
  tf2::Quaternion tfQuat_body;
  tf2::fromMsg(pose.pose.pose.orientation, tfQuat_body);
  tf2::Matrix3x3 m_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  m_body.getRPY(roll_body, pitch_body, yaw_body);

  body2world_R_matrix = Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch_body, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll_body, Eigen::Vector3d::UnitX());
  body2world_R_quat = Eigen::Quaterniond(body2world_R_matrix);
  
  if (this->airsim_flag_ == true)
  {
  // ! AirSim Simulation
  yaw_gimbal += yaw_body;
  pitch_gimbal += pitch_body;
  roll_gimbal = 0.0; // ? Fixed with Drone
  // ! ---------------------
  }

  gimbal2body_R_matrix = Eigen::AngleAxisd(yaw_gimbal, -Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch_gimbal, -Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll_gimbal, Eigen::Vector3d::UnitX()); 
  gimbal2body_R_quat = Eigen::Quaterniond(gimbal2body_R_matrix);
  gimbal_q_ = gimbal2body_R_quat;

  camera_pos_ = body2world_R_matrix * gimbal2body_pos_ + body2world_pos_;
  camera_q_ = body2world_R_quat * gimbal_q_;

  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;
  
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
  if (img.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);
  
  auto t1 = node_->now();
  // generate point cloud, update map
  proessDepthImage();

  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_, false);
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = node_->now();
  fuse_time_ += (t2 - t1).seconds();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).seconds());
  fuse_num_ += 1;
  if (show_occ_time_)
    RCLCPP_WARN(rclcpp::get_logger("flyco.plan_env"), "Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).seconds(), fuse_time_ / fuse_num_, max_fuse_time_);  

  return;
}

void MapROS::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                               const nav_msgs::msg::Odometry::ConstSharedPtr& pose) 
{
  // ! AirSim Simulation
  if (pose->child_frame_id == "X" || pose->child_frame_id == "O") return;
  // ! ---------------------

  sensor_msgs::msg::Image cur_img = *img;
  nav_msgs::msg::Odometry cur_pose = *pose;
  this->buffer_depth_dp_.push_back(cur_img);
  this->buffer_pose_dp_.push_back(cur_pose);

  return;
}

void MapROS::updateMapDepthPose()
{
  if (this->buffer_depth_dp_.empty() || this->buffer_pose_dp_.empty()) return;

  sensor_msgs::msg::Image img = this->buffer_depth_dp_.back();
  nav_msgs::msg::Odometry pose = this->buffer_pose_dp_.back();
  
  Eigen::Vector3d body2world_pos_, gimbal2body_pos_;
  Eigen::Matrix3d body2world_R_matrix, gimbal2body_R_matrix;
  Eigen::Quaterniond body2world_R_quat, gimbal2body_R_quat;

  body2world_pos_(0) = pose.pose.pose.position.x; // in world coordinate
  body2world_pos_(1) = pose.pose.pose.position.y; // in world coordinate
  body2world_pos_(2) = pose.pose.pose.position.z; // in world coordinate
  gimbal2body_pos_ = this->d2body_t_vector_;       // hyper param in body
                              
  tf2::Quaternion tfQuat_body;
  tf2::fromMsg(pose.pose.pose.orientation, tfQuat_body);
  tf2::Matrix3x3 m_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  m_body.getRPY(roll_body, pitch_body, yaw_body);

  body2world_R_matrix = Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch_body, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll_body, Eigen::Vector3d::UnitX());
  body2world_R_quat = Eigen::Quaterniond(body2world_R_matrix);

  camera_pos_ = body2world_R_matrix * gimbal2body_pos_ + body2world_pos_;
  camera_q_ = body2world_R_quat;

  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;
  
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
  if (img.encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);
  
  auto t1 = node_->now();
  // generate point cloud, update map
  proessDepthImage();

  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_, false);
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = node_->now();
  fuse_time_ += (t2 - t1).seconds();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).seconds());
  fuse_num_ += 1;
  if (show_occ_time_)
    RCLCPP_WARN(rclcpp::get_logger("flyco.plan_env"), "Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).seconds(), fuse_time_ / fuse_num_, max_fuse_time_);  

  return;
}

/**
 * @brief Callback function for cloud and pose messages.
 *
 * This function is called when new cloud and pose messages are received.
 * It processes the received data and updates the map accordingly.
 * Data interface function
 *
 * @param cloud The point cloud message in [world] frame.
 * @param pose  The body pose message in [world] frame.
 */
void MapROS::cloudPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
                               const nav_msgs::msg::Odometry::ConstSharedPtr& pose) 
{
  // ! AirSim Simulation
  if (pose->child_frame_id == "X" || pose->child_frame_id == "O") return;
  // ! ---------------------

  if (cloud->data.empty()) return;

  sensor_msgs::msg::PointCloud2 cur_cloud = *cloud;
  nav_msgs::msg::Odometry cur_pose = *pose;
  this->buffer_cloud_cp_.push_back(cur_cloud);
  this->buffer_pose_cp_.push_back(cur_pose);

  return;
}

void MapROS::updateMapCloudPose()
{
  if ((int)this->buffer_cloud_cp_.size() == 0 || (int)this->buffer_pose_cp_.size() == 0) return;
  
  sensor_msgs::msg::PointCloud2 cur_cloud = this->buffer_cloud_cp_.back();
  nav_msgs::msg::Odometry cur_pose = this->buffer_pose_cp_.back();
  
  // * Process Odometry
  tf2::Quaternion tfQuat_body;
  tf2::fromMsg(cur_pose.pose.pose.orientation, tfQuat_body);
  tf2::Matrix3x3 m_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  m_body.getRPY(roll_body, pitch_body, yaw_body);

  Eigen::Matrix3d body2world_R_matrix;

  body2world_R_matrix = Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch_body, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll_body, Eigen::Vector3d::UnitX());

  Eigen::Vector3d body2world_t_vector;
  body2world_t_vector << cur_pose.pose.pose.position.x, cur_pose.pose.pose.position.y, cur_pose.pose.pose.position.z;

  Eigen::Matrix3d sensor2world_R_matrix;
  Eigen::Vector3d sensor2world_t_vector;

  sensor2world_R_matrix = body2world_R_matrix * this->d2body_R_matrix_;
  sensor2world_t_vector = body2world_R_matrix * this->d2body_t_vector_ + body2world_t_vector;
  camera_pos_ = sensor2world_t_vector;

  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;

  // * Process Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cur_cloud, *temp_cloud);

  pcl::PointCloud<pcl::PointXYZ> world_cloud = *temp_cloud;
  int num = world_cloud.points.size();
  pcl::PointCloud<pcl::PointXYZ> air_cloud = generateAirPointCloud(world_cloud, sensor2world_R_matrix, sensor2world_t_vector);
  int air_num = air_cloud.points.size();

  // * Single Frame Point Cloud from LiDAR in World Frame
  int total_num = num + air_num;
  point_cloud_.points.resize(total_num);
  point_cloud_ = world_cloud + air_cloud;
  proj_points_cnt = total_num;

  // * Incrementally Update Map
  if (air_num > 0) map_->inputPointCloud(air_cloud, air_num, camera_pos_, true);
  if (num > 0) map_->inputPointCloud(world_cloud, num, camera_pos_, false);
  if (local_updated_) 
  {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  return;
}

/**
 * @brief Callback function for cloud and pose messages.
 *
 * This function is called when new cloud and pose messages are received.
 * It processes the received data and updates the map accordingly.
 * Data interface function
 *
 * @param cloud The point cloud message in [world] frame.
 * @param pose  The body pose message in [world] frame.
 */

void MapROS::cloudPoseTrackCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
                                    const nav_msgs::msg::Odometry::ConstSharedPtr& pose)
{
  if (cloud->data.empty()) return;

  sensor_msgs::msg::PointCloud2 cur_cloud = *cloud;
  nav_msgs::msg::Odometry cur_pose = *pose;
  this->buffer_all_ctep_.push_back(cur_cloud);
  this->buffer_pose_track_.push_back(cur_pose);

  return;
}

/**
 * @brief Callback function for processing target, environment, and pose messages from Tracking module.
 *
 * This function is called when new target, environment, and pose messages are received.
 * It processes the received data and updates the map accordingly.
 *
 * @param target The target point cloud message in [world] frame.      --> Note: from Tracking publisher
 * @param env    The environment point cloud message in [world] frame. --> Note: from Tracking publisher
 * @param pose   The body pose message in [world] frame.               --> Note: from Tracking publisher
 */
void MapROS::tarEnvPoseCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& target,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& env,
                                const nav_msgs::msg::Odometry::ConstSharedPtr& pose)
{
  if (target->data.empty() || env->data.empty()) return;

  sensor_msgs::msg::PointCloud2 cur_target = *target;
  sensor_msgs::msg::PointCloud2 cur_env = *env;
  nav_msgs::msg::Odometry cur_pose = *pose;
  this->buffer_target_ctep_.push_back(cur_target);
  this->buffer_env_ctep_.push_back(cur_env);
  this->buffer_pose_ctep_.push_back(cur_pose);

  return;
}

void MapROS::updateMapCloudTarEnvPose()
{
  // * Process perceived cloud and pose
  if (this->buffer_all_ctep_.empty() || this->buffer_pose_track_.empty()) return;

  sensor_msgs::msg::PointCloud2 cloud = this->buffer_all_ctep_.back();
  nav_msgs::msg::Odometry pose_track = this->buffer_pose_track_.back();

  tf2::Quaternion tfQuat_body;
  tf2::fromMsg(pose_track.pose.pose.orientation, tfQuat_body);
  tf2::Matrix3x3 m_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  m_body.getRPY(roll_body, pitch_body, yaw_body);

  Eigen::Matrix3d body2world_R_matrix;
  body2world_R_matrix = Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch_body, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll_body, Eigen::Vector3d::UnitX());
  
  Eigen::Vector3d body2world_t_vector;
  body2world_t_vector << pose_track.pose.pose.position.x, pose_track.pose.pose.position.y, pose_track.pose.pose.position.z;

  Eigen::Matrix3d sensor2world_R_matrix;
  Eigen::Vector3d sensor2world_t_vector;
  sensor2world_R_matrix = body2world_R_matrix * this->d2body_R_matrix_;
  sensor2world_t_vector = body2world_R_matrix * this->d2body_t_vector_ + body2world_t_vector;
  camera_pos_ = sensor2world_t_vector;

  if (!map_->isInMap(camera_pos_)) return;
  
  // ? Process Point Cloud
  pcl::PointCloud<pcl::PointXYZ> all_cloud;
  pcl::fromROSMsg(cloud, all_cloud);

  pcl::PointCloud<pcl::PointXYZ> air_cloud = generateAirPointCloud(all_cloud, sensor2world_R_matrix, sensor2world_t_vector);

  int all_num = all_cloud.points.size();
  int air_num = air_cloud.points.size();

  // ? Single Frame Point Cloud from LiDAR in World Frame
  int total_num = all_num + air_num;
  point_cloud_.points.resize(total_num);
  point_cloud_ = all_cloud + air_cloud;
  proj_points_cnt = total_num;

  // ? Incrementally Update Map
  if (air_num > 0) map_->inputPointCloud(air_cloud, air_num, camera_pos_, true);
  if (all_num > 0) map_->inputPointCloud(all_cloud, all_num, camera_pos_, false);
  if (local_updated_) 
  {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  // * Process tracking labels
  if (this->buffer_target_ctep_.empty() || this->buffer_env_ctep_.empty() || this->buffer_pose_ctep_.empty()) return;

  sensor_msgs::msg::PointCloud2 target = this->buffer_target_ctep_.back();
  sensor_msgs::msg::PointCloud2 env = this->buffer_env_ctep_.back();
  nav_msgs::msg::Odometry pose = this->buffer_pose_ctep_.back();
  
  // * Process Odometry
  tf2::Quaternion quat_track;
  tf2::fromMsg(pose.pose.pose.orientation, quat_track);
  tf2::Matrix3x3 m_track(quat_track);
  double roll_track, pitch_track, yaw_track;
  m_track.getRPY(roll_track, pitch_track, yaw_track);

  Eigen::Matrix3d body2world_R_track;
  body2world_R_track = Eigen::AngleAxisd(yaw_track, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(pitch_track, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(roll_track, Eigen::Vector3d::UnitX());

  Eigen::Vector3d body2world_t_track;
  body2world_t_track << pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z;

  Eigen::Matrix3d sensor2world_R_track;
  Eigen::Vector3d sensor2world_t_track;

  sensor2world_R_track = body2world_R_track * this->d2body_R_matrix_;
  sensor2world_t_track = body2world_R_track * this->d2body_t_vector_ + body2world_t_track;
  Eigen::Vector3d cam_pos_track = sensor2world_t_track;

  if (!map_->isInMap(cam_pos_track)) return;

  // ? Process Point Cloud
  pcl::PointCloud<pcl::PointXYZ> target_cloud, env_cloud;
  pcl::fromROSMsg(target, target_cloud);
  pcl::fromROSMsg(env, env_cloud);

  // cout << "cur input target size : " << target_cloud.points.size() << endl;

  int target_num = target_cloud.points.size();
  int env_num = env_cloud.points.size();

  // ? Incrementally Update Map
  if (target_num > 0) map_->clearAndMapTarget(target_cloud, target_num, cam_pos_track);
  if (env_num > 0) map_->clearAndMapEnvironment(env_cloud, env_num, cam_pos_track);

  return;
}

void MapROS::proessDepthImage() 
{
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) 
  {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) 
    {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      Eigen::Vector3d normal_uvd = Eigen::Vector3d(u, v, 1.0);
      Eigen::Vector3d normal_xyz = K_depth_.inverse() * normal_uvd;
      double length = normal_xyz.norm();
      Eigen::Vector3d xyz = normal_xyz / length * depth;
      if (this->airsim_flag_ == true)
      {
        // ! AirSim Simulation
        pt_cur << xyz(2), -xyz(0), -xyz(1);
        // ! --------------------
      }
      else
      {
        // ! Real-world experiment
        pt_cur << xyz(0), xyz(1), -xyz(2);
        // ! --------------------
      }
      pt_world = camera_r * pt_cur + camera_pos_;
      
      auto& pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

  return;
}

pcl::PointCloud<pcl::PointXYZ> MapROS::generateAirPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& input_cloud, 
    const Eigen::Matrix3d& R_sensor2world, 
    const Eigen::Vector3d& t_sensor2world)
{
    pcl::PointCloud<pcl::PointXYZ> air_inf_cloud;

    const double horizontal_fov = this->h_fov_;
    const double vertical_fov_min = this->v_fov_min_ + this->inf_fov_;
    const double vertical_fov_max = this->v_fov_max_ - this->inf_fov_;
    const double horizontal_resolution = this->h_res_;
    const double vertical_resolution = this->v_res_;
    const double max_depth = this->mapping_visible_dist_ + 0.5;
    const double min_depth = 0.5;

    int cols = static_cast<int>(horizontal_fov / horizontal_resolution);
    int rows = static_cast<int>((vertical_fov_max - vertical_fov_min) / vertical_resolution);

    Eigen::MatrixXd depth_map = Eigen::MatrixXd::Constant(rows, cols, max_depth);
    double horizontal_fov_min = -horizontal_fov / 2.0;

    for (const auto& pt : input_cloud.points) 
    {
      Eigen::Vector3d pt_world(pt.x, pt.y, pt.z);
      Eigen::Vector3d pt_sensor = R_sensor2world.transpose() * (pt_world - t_sensor2world);

      
      double horizontal_angle = std::atan2(pt_sensor.y(), pt_sensor.x()) * 180.0 / M_PI;
      double vertical_angle = std::atan2(pt_sensor.z(), std::sqrt(pt_sensor.x() * pt_sensor.x() + pt_sensor.y() * pt_sensor.y())) * 180.0 / M_PI;

      if (horizontal_angle < horizontal_fov_min || horizontal_angle > -horizontal_fov_min ||
          vertical_angle < vertical_fov_min || vertical_angle > vertical_fov_max) continue;

      int u = static_cast<int>((horizontal_angle - horizontal_fov_min) / horizontal_resolution);
      int v = static_cast<int>((vertical_angle - vertical_fov_min) / vertical_resolution);

      if (u >= 0 && u < cols && v >= 0 && v < rows) 
      {
        double depth = pt_sensor.norm();
        depth_map(v, u) = std::min(depth_map(v, u), depth);
      }
    }

    int dilation_radius = this->dil_pixels_;
    Eigen::MatrixXd depth_map_dilated = depth_map;

    for (int v = 0; v < rows; v++) 
    {
      for (int u = 0; u < cols; u++) 
      {
        if (depth_map(v, u) < max_depth && depth_map(v, u) >= min_depth) 
        {
          for (int dv = -dilation_radius; dv <= dilation_radius; dv++) 
          {
            for (int du = -dilation_radius; du <= dilation_radius; du++) 
            {
              int u_neighbor = u + du;
              int v_neighbor = v + dv;

              if (u_neighbor >= 0 && u_neighbor < cols && v_neighbor >= 0 && v_neighbor < rows) 
              {
                if (depth_map(v_neighbor, u_neighbor) < max_depth && depth_map(v_neighbor, u_neighbor) >= min_depth) continue;
                depth_map_dilated(v_neighbor, u_neighbor) = 0.9*min_depth;
              }
            }
          }
        }
      }
    }

    for (int v = 0; v < rows; v++) 
    {
      for (int u = 0; u < cols; u++) 
      {
        double depth = depth_map_dilated(v, u);

        if (depth < max_depth) continue;

        if (depth >= max_depth) depth = max_depth;

        double horizontal_angle = horizontal_fov_min + u * horizontal_resolution;
        double vertical_angle = vertical_fov_min + v * vertical_resolution;

        Eigen::Vector3d direction;
        direction.x() = std::cos(horizontal_angle * M_PI / 180.0) * std::cos(vertical_angle * M_PI / 180.0);
        direction.y() = std::sin(horizontal_angle * M_PI / 180.0) * std::cos(vertical_angle * M_PI / 180.0);
        direction.z() = std::sin(vertical_angle * M_PI / 180.0);

        Eigen::Vector3d pt_sensor = direction * depth;

        Eigen::Vector3d pt_world = R_sensor2world * pt_sensor + t_sensor2world;

        pcl::PointXYZ pt;
        pt.x = pt_world.x();
        pt.y = pt_world.y();
        pt.z = pt_world.z();
        air_inf_cloud.points.push_back(pt);
      }
    }

    air_inf_cloud.width = air_inf_cloud.points.size();
    air_inf_cloud.height = 1;
    air_inf_cloud.is_dense = true;

    return air_inf_cloud;
}

void MapROS::publishMapAll() 
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2, cloud_unknown, cloud_free;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
        Eigen::Vector3d pos;
        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
        
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) 
        {
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
 
          // ! Visualization of Occupied Cells away from the environment
   
          // int step = ceil(1.0 / map_->mp_->resolution_);
          // bool add = true;
          // for (int dx = -step; dx <= step; ++dx)
          //   for (int dy = -step; dy <= step; ++dy)
          //     for (int dz = -step; dz <= step; ++dz)
          //     {
          //       if (dx == 0 && dy == 0 && dz == 0) continue;
          //       Eigen::Vector3i pos_neighbor = Eigen::Vector3i(x + dx, y + dy, z + dz);
          //       if (map_->md_->env_buffer_[map_->toAddress(pos_neighbor)] == 1)
          //       {
          //         add = false;
          //         break;
          //       }
          //     }
          
          // if (!add) continue;
          
          // ! Visualization of Occupied Cells away from the environment

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }

        // if (map_->getOccupancy(Eigen::Vector3i(x, y, z)) == SDFMap::OCCUPANCY::UNKNOWN)
        // {
        //   if (pos(2) > visualization_truncate_height_) continue;
        //   if (pos(2) < visualization_truncate_low_) continue;
        //   pt.x = pos(0);
        //   pt.y = pos(1);
        //   pt.z = pos(2);
        //   cloud_unknown.push_back(pt);
        // }

        // if (map_->getOccupancy(Eigen::Vector3i(x, y, z)) == SDFMap::OCCUPANCY::FREE)
        // {
        //   if (pos(2) > visualization_truncate_height_) continue;
        //   if (pos(2) < visualization_truncate_low_) continue;
        //   pt.x = pos(0);
        //   pt.y = pos(1);
        //   pt.z = pos(2);
        //   // if (pt.z > 10.0 && pt.z < 13.0) cloud_free.push_back(pt);
        //   cloud_free.push_back(pt);
        // }

        if (this->inflate_vis_ == false) continue;

        if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1)
        {
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud2.push_back(pt);
        }
      }
  if ((int)cloud1.points.size() == 0) return;

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;

  cloud_unknown.width = cloud_unknown.points.size();
  cloud_unknown.height = 1;
  cloud_unknown.is_dense = true;
  cloud_unknown.header.frame_id = frame_id_;

  cloud_free.width = cloud_free.points.size();
  cloud_free.height = 1;
  cloud_free.is_dense = true;
  cloud_free.header.frame_id = frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg, inflate_cloud_msg, unknown_cloud_msg, free_cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  this->map_all_pub_->publish(cloud_msg);

  pcl::toROSMsg(cloud_unknown, unknown_cloud_msg);
  // this->map_unknown_pub_->publish(unknown_cloud_msg);
  pcl::toROSMsg(cloud_free, free_cloud_msg);
  // this->map_free_pub_->publish(free_cloud_msg);

  if (this->inflate_vis_ == true)
  {
    if ((int)cloud2.points.size() == 0) return;
    pcl::toROSMsg(cloud2, inflate_cloud_msg);
    this->map_all_inflate_pub_->publish(inflate_cloud_msg);
  }

  return;
}

void MapROS::publishMapLocal() 
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) 
        {
          // Occupied cells
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }

        if (this->inflate_vis_ == false) continue;

        if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1)
        {
          // Inflated cells
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud2.push_back(pt);
        }
      }
  
  if ((int)cloud.points.size() == 0) return;

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg, inflate_cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_local_pub_->publish(cloud_msg);
  if (this->inflate_vis_ == true)
  {
    if ((int)cloud2.points.size() == 0) return;
    pcl::toROSMsg(cloud2, inflate_cloud_msg);
    map_local_inflate_pub_->publish(inflate_cloud_msg);
  }

  return;
}

void MapROS::publishDepth() 
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < proj_points_cnt; ++i) 
  {
    cloud.push_back(point_cloud_.points[i]);
  }
  if ((int)cloud.points.size() == 0) return;

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_->publish(cloud_msg);

  return;
}

void MapROS::publishMappingRange()
{
  Eigen::Vector3d box_min, box_max;
  map_->getGlobalBox(box_min, box_max);

  visualization_msgs::msg::MarkerArray marker_array;

  // * Edge of Mapping Range
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = frame_id_;
  line_marker.header.stamp = node_->now();
  line_marker.ns = "mapping_range";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = 0.5;
  line_marker.color.a = 1.0;
  line_marker.color.r = 1.0;
  line_marker.color.g = 0.0;
  line_marker.color.b = 0.0;

  geometry_msgs::msg::Point p[8];
  p[0].x = box_min.x(); p[0].y = box_min.y(); p[0].z = box_min.z();
  p[1].x = box_max.x(); p[1].y = box_min.y(); p[1].z = box_min.z();
  p[2].x = box_max.x(); p[2].y = box_max.y(); p[2].z = box_min.z();
  p[3].x = box_min.x(); p[3].y = box_max.y(); p[3].z = box_min.z();
  p[4].x = box_min.x(); p[4].y = box_min.y(); p[4].z = box_max.z();
  p[5].x = box_max.x(); p[5].y = box_min.y(); p[5].z = box_max.z();
  p[6].x = box_max.x(); p[6].y = box_max.y(); p[6].z = box_max.z();
  p[7].x = box_min.x(); p[7].y = box_max.y(); p[7].z = box_max.z();

  vector<geometry_msgs::msg::Point> points = 
  {
    p[0], p[1], p[1], p[2], p[2], p[3], p[3], p[0],
    p[4], p[5], p[5], p[6], p[6], p[7], p[7], p[4],
    p[0], p[4], p[1], p[5], p[2], p[6], p[3], p[7]
  };

  line_marker.points.insert(line_marker.points.end(), points.begin(), points.end());
  marker_array.markers.push_back(line_marker);

  // * Cube of Mapping Range
  visualization_msgs::msg::Marker cube_marker;
  cube_marker.header.frame_id = frame_id_;
  cube_marker.header.stamp = node_->now();
  cube_marker.ns = "mapping_range";
  cube_marker.id = 1;
  cube_marker.type = visualization_msgs::msg::Marker::CUBE;
  cube_marker.action = visualization_msgs::msg::Marker::ADD;
  cube_marker.pose.position.x = 0.5 * (box_min.x() + box_max.x());
  cube_marker.pose.position.y = 0.5 * (box_min.y() + box_max.y());
  cube_marker.pose.position.z = 0.5 * (box_min.z() + box_max.z());
  cube_marker.pose.orientation.w = 1.0;
  cube_marker.scale.x = box_max.x() - box_min.x();
  cube_marker.scale.y = box_max.y() - box_min.y();
  cube_marker.scale.z = box_max.z() - box_min.z();

  cube_marker.color.r = 0.0;
  cube_marker.color.g = 1.0;
  cube_marker.color.b = 0.5;
  cube_marker.color.a = 0.3;

  marker_array.markers.push_back(cube_marker);

  this->mapping_range_pub_->publish(marker_array);

  return;
}

void MapROS::publishUpdateRange() 
{
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::msg::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = node_->now();
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_->publish(mk);

  return;
}

void MapROS::publishTarget()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {

        if (map_->md_->target_buffer_[map_->toAddress(x, y, z)] == 1)
        {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
  
  if ((int)cloud1.points.size() == 0) return;

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  this->target_pub_->publish(cloud_msg);

  return;
}

void MapROS::publishEnvironment()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {

        if (map_->md_->env_buffer_[map_->toAddress(x, y, z)] == 1 && map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_)
        {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
  
  if ((int)cloud1.points.size() == 0) return;

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  this->env_pub_->publish(cloud_msg);

  return;
}

void MapROS::publishESDF() 
{
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = esdf_slice_height_;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  if ((int)cloud.points.size() == 0) return;

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_->publish(cloud_msg);

  return;
}
}
