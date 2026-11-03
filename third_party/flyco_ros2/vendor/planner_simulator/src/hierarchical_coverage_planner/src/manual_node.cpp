#include <rclcpp/create_timer.hpp>
#include <flyco_ros2/parameters.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
 * Description  :    This file is the main file of the manual mode of FlyCo.
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

#include "hierarchical_coverage_planner/manual_node.h"

namespace flyco
{

void Manual_Node::init(const rclcpp::Node::SharedPtr& nh)
{
  node_ = nh;
  this->suspension_target_ = flyco::parameter_or(nh, "hcplanner.suspension_rate", 99.0);
  this->ground_ = flyco::parameter_or(nh, "viewpoint_manager.GroundPos", -1.0);
  this->res_ = flyco::parameter_or(nh, "hcmap.resolution", 0.4);
  this->sample_size_ = flyco::parameter_or(nh, "hcplanner.mesh_sample_size", -1);
  vis_inflation_ = flyco::parameter_or(nh, "viewpoint_manager.vis_inf", 0.5);

  int num = 10;
  this->buffer_mesh_V_ = boost::circular_buffer<Eigen::MatrixXd>(num);
  this->buffer_mesh_F_ = boost::circular_buffer<Eigen::MatrixXi>(num);
  
  this->percep_utils_.reset(new PerceptionUtils);
  this->percep_utils_->init(nh);

  this->odom_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh, "/flyco_planner/body_pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  this->gimbal_sub_.reset(new message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>(nh, "/flyco_planner/gimbal_pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  this->sync_odom_gimbal_.reset(new message_filters::Synchronizer<SyncPolicyPoseGimbal>(SyncPolicyPoseGimbal(20), *this->odom_sub_, *this->gimbal_sub_));
  this->sync_odom_gimbal_->registerCallback(std::bind(&Manual_Node::execCallback, this, std::placeholders::_1, std::placeholders::_2));

  this->manual_traj_pub_ = nh->create_publisher<nav_msgs::msg::Path>("/joy_ctrl/task_traj", rclcpp::QoS(1).reliable().transient_local());
  this->manual_vp_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("/joy_ctrl/viewpoints", rclcpp::QoS(1).reliable().transient_local());
  this->manual_mesh_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("/joy_ctrl/mesh", rclcpp::QoS(1).reliable().transient_local());
  this->manual_uncovered_pub_ = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/joy_ctrl/uncovered", rclcpp::QoS(1).reliable());

  this->manual_vis_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.05), std::bind(&Manual_Node::manualVisCallback, this));
  this->manual_sta_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&Manual_Node::statisticsCallback, this));
  this->mesh_sub_ = nh->create_subscription<quadrotor_msgs::msg::Mesh>("/flyco_planner/predicted_mesh", rclcpp::QoS(1).reliable(), std::bind(&Manual_Node::meshCallback, this, std::placeholders::_1));
  this->joy_sub_ = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::QoS(50).reliable(), std::bind(&Manual_Node::joyCallback, this, std::placeholders::_1));
  
  return;
}

} // namespace flyco
