#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <flyco_ros2/parameters.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
 * Date         :    Apr. 2025
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the implementation file of the RGB-D mapping in FlyCo.
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

#include "rgb_point_map/rgb_point_map.h"

namespace flyco
{

void RGBPointMap::init(const rclcpp::Node::SharedPtr& nh)
{
  node_ = nh;
  this->voxel_size_ = flyco::parameter_or(nh, "rgb_mapping.voxel_size", 0.1f);
  this->chunk_size_ = flyco::parameter_or(nh, "rgb_mapping.chunk_size", 32);
  this->origin_x_ = flyco::parameter_or(nh, "rgb_mapping.origin_x", -20.0f);
  this->origin_y_ = flyco::parameter_or(nh, "rgb_mapping.origin_y", -100.0f);
  this->origin_z_ = flyco::parameter_or(nh, "rgb_mapping.origin_z", 0.0f);
  this->map_size_x_ = flyco::parameter_or(nh, "rgb_mapping.map_size_x", 100.0f);
  this->map_size_y_ = flyco::parameter_or(nh, "rgb_mapping.map_size_y", 100.0f);
  this->map_size_z_ = flyco::parameter_or(nh, "rgb_mapping.map_size_z", 100.0f);
  this->en_inherit_ = flyco::parameter_or(nh, "rgb_mapping.en_inherit", false);
  this->fx_ = flyco::parameter_or(nh, "rgb_mapping.fx", 0.0);
  this->fy_ = flyco::parameter_or(nh, "rgb_mapping.fy", 0.0);
  this->cx_ = flyco::parameter_or(nh, "rgb_mapping.cx", 0.0);
  this->cy_ = flyco::parameter_or(nh, "rgb_mapping.cy", 0.0);
  this->img_rows_ = flyco::parameter_or(nh, "rgb_mapping.img_rows", 480);
  this->img_cols_ = flyco::parameter_or(nh, "rgb_mapping.img_cols", 640);
  this->fov_scale_ = flyco::parameter_or(nh, "rgb_mapping.fov_scale", 1.0);
  this->c2b_x_ = flyco::parameter_or(nh, "rgb_mapping.c2b_x", 0.0);
  this->c2b_y_ = flyco::parameter_or(nh, "rgb_mapping.c2b_y", 0.0);
  this->c2b_z_ = flyco::parameter_or(nh, "rgb_mapping.c2b_z", 0.0);
  this->c2b_roll_ = flyco::parameter_or(nh, "rgb_mapping.c2b_roll", 0.0);
  this->c2b_pitch_ = flyco::parameter_or(nh, "rgb_mapping.c2b_pitch", 0.0);
  this->c2b_yaw_ = flyco::parameter_or(nh, "rgb_mapping.c2b_yaw", 0.0);
  this->gt_address_ = flyco::parameter_or(nh, "rgb_mapping.model", string(""));
  this->voxel_num_x_ = static_cast<int>(this->map_size_x_ / this->voxel_size_);
  this->voxel_num_y_ = static_cast<int>(this->map_size_y_ / this->voxel_size_);
  this->voxel_num_z_ = static_cast<int>(this->map_size_z_ / this->voxel_size_);
  this->save_map_ = false;
  this->already_save_ = false;

  // pre-computation
  this->cam2body_R_matrix_ = Eigen::AngleAxisd(this->c2b_roll_, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(this->c2b_pitch_, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(this->c2b_yaw_, Eigen::Vector3d::UnitZ());
  this->cam2body_t_vector_ = Eigen::Vector3d(this->c2b_x_, this->c2b_y_, this->c2b_z_);

  this->img_3d_table_.resize(this->img_rows_, vector<Eigen::Vector3d>(this->img_cols_));
  for (int v = 0; v < this->img_rows_; v++)
    for (int u = 0; u < this->img_cols_; u++)
    {
      this->img_3d_table_[v][u](0) = (u - this->cx_) * this->fov_scale_ / this->fx_;
      this->img_3d_table_[v][u](1) = (v - this->cy_) * this->fov_scale_ / this->fy_;
      this->img_3d_table_[v][u](2) = this->fov_scale_;
    }
  this->fov_horizontal_ = 2.0 * atan((double)this->img_cols_ / (2.0 * this->fx_));
  this->fov_vertical_ = 2.0 * atan((double)this->img_rows_ / (2.0 * this->fy_));

  double hor = this->fov_scale_ * tan(this->fov_horizontal_ / 2.0);
  double ver = this->fov_scale_ * tan(this->fov_vertical_ / 2.0);
  Eigen::Vector3d origin(0, 0, 0);
  Eigen::Vector3d left_up(this->fov_scale_, hor, ver);
  Eigen::Vector3d left_down(this->fov_scale_, hor, -ver);
  Eigen::Vector3d right_up(this->fov_scale_, -hor, ver);
  Eigen::Vector3d right_down(this->fov_scale_, -hor, -ver);

  this->cam_vertices1_.push_back(origin);
  this->cam_vertices2_.push_back(left_up);
  this->cam_vertices1_.push_back(origin);
  this->cam_vertices2_.push_back(left_down);
  this->cam_vertices1_.push_back(origin);
  this->cam_vertices2_.push_back(right_up);
  this->cam_vertices1_.push_back(origin);
  this->cam_vertices2_.push_back(right_down);

  this->cam_vertices1_.push_back(left_up);
  this->cam_vertices2_.push_back(right_up);
  this->cam_vertices1_.push_back(right_up);
  this->cam_vertices2_.push_back(right_down);
  this->cam_vertices1_.push_back(right_down);
  this->cam_vertices2_.push_back(left_down);
  this->cam_vertices1_.push_back(left_down);
  this->cam_vertices2_.push_back(left_up);

  if (this->en_inherit_) this->loadInheritMap();

  this->start_mapping_ = false;
  this->total_rgb_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  this->img_pixels_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  this->buffer_rgbd_ = boost::circular_buffer<sensor_msgs::msg::PointCloud2>(20);
  this->buffer_rgbd_pose_ = boost::circular_buffer<nav_msgs::msg::Odometry>(20);
  this->buffer_img_ = boost::circular_buffer<sensor_msgs::msg::Image>(50);
  this->buffer_pose_ = boost::circular_buffer<nav_msgs::msg::Odometry>(50);
  this->buffer_quat_ = boost::circular_buffer<geometry_msgs::msg::QuaternionStamped>(50);
  this->buffer_cur_pose_ = boost::circular_buffer<nav_msgs::msg::Odometry>(50);
  
  this->image_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::Image>(nh, "/rgb_mapping/img", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  this->pose_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh, "/rgb_mapping/body_pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  this->gimbal_sub_.reset(new message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>(nh, "/rgb_mapping/gimbal_pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  this->sync_image_pose_gimbal_.reset(new message_filters::Synchronizer<SyncPolicyImagePoseGimbal>(
      SyncPolicyImagePoseGimbal(20), *this->image_sub_, *this->pose_sub_, *this->gimbal_sub_));
  this->sync_image_pose_gimbal_->registerCallback(std::bind(&RGBPointMap::igoCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  this->rgbd_sub_.reset(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(nh, "/rgb_mapping/rgbd", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  this->rgbd_pose_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh, "/rgb_mapping/body_pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  this->sync_rgbd_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(
      SyncPolicyCloudPose(20), *this->rgbd_sub_, *this->rgbd_pose_sub_));
  this->sync_rgbd_pose_->registerCallback(std::bind(&RGBPointMap::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  this->joy_sub_ = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::QoS(50).reliable(), std::bind(&RGBPointMap::joyCallback, this, std::placeholders::_1));
  this->odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>("/rgb_mapping/body_pose", rclcpp::QoS(1).reliable(), std::bind(&RGBPointMap::odomCallback, this, std::placeholders::_1));
  // this->trigger_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", rclcpp::QoS(50).reliable(), std::bind(&RGBPointMap::triggerCallback, this, std::placeholders::_1));
  // this->pred_mesh_sub_ =  nh->create_subscription<quadrotor_msgs::msg::Mesh>("/prediction/predicted_mesh", rclcpp::QoS(1).reliable(), std::bind(&RGBPointMap::meshCallback, this, std::placeholders::_1));

  this->mesh_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(this->gt_address_, *this->mesh_cloud_) == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("flyco.rgb_point_map"), "Failed to load ground truth mesh from %s", this->gt_address_.c_str());
  }

  this->mesh_tree_.setInputCloud(this->mesh_cloud_);

  this->map_pub_ = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/rgb_mapping/rgb_map", rclcpp::QoS(1).reliable());
  this->img_3d_pub_ = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/rgb_mapping/img_3d", rclcpp::QoS(1).reliable());
  this->fov_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/rgb_mapping/fov", rclcpp::QoS(1).reliable());

  this->mapping_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.2), std::bind(&RGBPointMap::mappingCallback, this));
  this->publish_map_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.2), std::bind(&RGBPointMap::publishMapCallback, this));
  this->publish_cam_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.03), std::bind(&RGBPointMap::publishCamCallback, this));
  this->save_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.2), std::bind(&RGBPointMap::saveMapCallback, this));

  return;
}

void RGBPointMap::start()
{
  this->start_mapping_ = true;
  RCLCPP_INFO(rclcpp::get_logger("flyco.rgb_point_map"), "\033[42;37m[RGBPointMap]\033[47;32m Start Mapping! \033[0m");
  return;
}

void RGBPointMap::mappingCallback()
{
  if (this->buffer_rgbd_.empty()) return;

  sensor_msgs::msg::PointCloud2 msg = this->buffer_rgbd_.back();
  nav_msgs::msg::Odometry pose_msg = this->buffer_rgbd_pose_.back();

  Eigen::Vector3d cam_pos(pose_msg.pose.pose.position.x,
                          pose_msg.pose.pose.position.y,
                          pose_msg.pose.pose.position.z); 

  pcl::PointCloud<pcl::PointXYZRGB> single_frame_rgb_cloud;
  pcl::fromROSMsg(msg, single_frame_rgb_cloud);

  this->insertPointCloudIncremental(std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(single_frame_rgb_cloud), cam_pos, false);
  auto filtered = this->exportVoxelCloud();

  *this->total_rgb_cloud_ = *filtered;
  this->total_rgb_cloud_->width = this->total_rgb_cloud_->points.size();
  this->total_rgb_cloud_->height = 1;
  this->total_rgb_cloud_->is_dense = true;

  return;
}

void RGBPointMap::publishMapCallback()
{
  if (!this->start_mapping_) return;
  if (this->total_rgb_cloud_ == nullptr) return;

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*this->total_rgb_cloud_, map_msg);
  map_msg.header.frame_id = "world";
  map_msg.header.stamp = node_->now();
  this->map_pub_->publish(map_msg);

  return;
}

void RGBPointMap::publishCamCallback()
{
  if (!this->start_mapping_) return;
  if (this->buffer_img_.empty() || this->buffer_cur_pose_.empty() || this->buffer_quat_.empty()) return;

  sensor_msgs::msg::Image img_msg = this->buffer_img_.back();
  nav_msgs::msg::Odometry pose_msg = this->buffer_cur_pose_.back();
  geometry_msgs::msg::QuaternionStamped quat_msg = this->buffer_quat_.back();

  // * get transformation
  Eigen::Matrix3d sensor2world_R_matrix;
  Eigen::Vector3d sensor2world_t_vector;

  Eigen::Matrix3d img2cam_R_matrix;
  img2cam_R_matrix << 0.0, 0.0, 1.0,
                      -1.0, 0.0, 0.0,
                      0.0, -1.0, 0.0;
  Eigen::Vector3d img2cam_t_vector = Eigen::Vector3d::Zero();

  Eigen::Matrix3d cam2body_R_matrix = Eigen::Matrix3d::Identity();
  Eigen::Vector3d cam2body_t_vector = Eigen::Vector3d::Zero();

  tf2::Quaternion tfQuat_body;
  tf2::fromMsg(pose_msg.pose.pose.orientation, tfQuat_body);
  tf2::Matrix3x3 p_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  p_body.getRPY(roll_body, pitch_body, yaw_body);

  tf2::Quaternion img_quat_tf(quat_msg.quaternion.x, quat_msg.quaternion.y, quat_msg.quaternion.z, quat_msg.quaternion.w);
  tf2::Matrix3x3 m_body(img_quat_tf);
  double roll, pitch, yaw;
  m_body.getRPY(roll, pitch, yaw);
  roll = 0.0;
  Eigen::Matrix3d body2world_R_matrix;
  body2world_R_matrix = Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  Eigen::Vector3d body2world_t_vector(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z);

  sensor2world_R_matrix = body2world_R_matrix * this->cam2body_R_matrix_ * img2cam_R_matrix;
  sensor2world_t_vector = body2world_R_matrix * this->cam2body_R_matrix_ * img2cam_t_vector + body2world_R_matrix * this->cam2body_t_vector_ + body2world_t_vector;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = sensor2world_R_matrix;
  T.block<3, 1>(0, 3) = sensor2world_t_vector;

  // * get Cam
  this->processImg(img_msg);
  pcl::transformPointCloud(*this->img_pixels_, *this->img_pixels_, T);

  sensor_msgs::msg::PointCloud2 img_3d_msg;
  pcl::toROSMsg(*this->img_pixels_, img_3d_msg);
  img_3d_msg.header.frame_id = "world";
  img_3d_msg.header.stamp = node_->now();
  this->img_3d_pub_->publish(img_3d_msg);

  Eigen::Vector3d fov_pos = sensor2world_t_vector;
  double fov_pitch = pitch;
  double fov_yaw = yaw_body;
  vector<Eigen::Vector3d> fov_list1, fov_list2;
  this->getFOV(fov_pos, fov_pitch, fov_yaw, fov_list1, fov_list2);

  visualization_msgs::msg::MarkerArray fov_markers;

  double fov_line_scale = 0.1;
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = node_->now();
  mk.id = 0;
  mk.ns = "rgb_mapping_fov";
  mk.type = visualization_msgs::msg::Marker::LINE_LIST;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 0.9;
  mk.scale.x = fov_line_scale;
  mk.scale.y = fov_line_scale;
  mk.scale.z = fov_line_scale;
  mk.pose.orientation.w = 1.0;

  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(fov_list1.size()); ++i) 
  {
    pt.x = fov_list1[i](0);
    pt.y = fov_list1[i](1);
    pt.z = fov_list1[i](2);
    mk.points.push_back(pt);

    pt.x = fov_list2[i](0);
    pt.y = fov_list2[i](1);
    pt.z = fov_list2[i](2);
    mk.points.push_back(pt);
  }

  fov_markers.markers.push_back(mk);
  this->fov_pub_->publish(fov_markers);

  return;
}

void RGBPointMap::saveMapCallback()
{
  if (!this->save_map_ || this->already_save_) return;

  this->already_save_ = true;

  string map_path = (ament_index_cpp::get_package_share_directory("flyco_ros2") + "/vendor/planner_simulator/src/rgb_point_map") + "/inherit_map";
  string rgbd_file = map_path + "/rgbd.pcd";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  rgbd_cloud = this->exportVoxelCloud();

  pcl::io::savePCDFileBinaryCompressed(rgbd_file, *rgbd_cloud);

  RCLCPP_INFO(rclcpp::get_logger("flyco.rgb_point_map"), "\033[42;37m[RGBPointMap]\033[47;32m Save Mapping! \033[0m");

  return;
}

}
