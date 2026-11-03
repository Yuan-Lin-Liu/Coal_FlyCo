#include <rclcpp/rclcpp.hpp>
#include <flyco_ros2/logging.hpp>
#include <geometry_msgs/msg/point.hpp>
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
 * Description  :    This file is the header file of Manual_Node class, which implements the
 *                   manual mode of FlyCo.
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

#ifndef _MANUAL_NODE_H_
#define _MANUAL_NODE_H_

#include "active_perception/perception_utils.h"
#include "plan_utils/visibility_st.hpp"
#include "quadrotor_msgs/msg/mesh.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <embree3/rtcore.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/random_sample.h>
#include <boost/circular_buffer.hpp>

using namespace std;

namespace flyco
{
  
  class PerceptionUtils;

  class Manual_Node
  {
  rclcpp::Node::SharedPtr node_;

    public:
      Manual_Node(){
      }
      ~Manual_Node(){
      }
      /* Func */
      void init(const rclcpp::Node::SharedPtr& nh);

    private:
      /* ROS Service */
      typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, geometry_msgs::msg::QuaternionStamped> SyncPolicyPoseGimbal;
      typedef unique_ptr<message_filters::Synchronizer<SyncPolicyPoseGimbal>> SynchronizerPoseGimbal;
      unique_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
      unique_ptr<message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>> gimbal_sub_;
      SynchronizerPoseGimbal sync_odom_gimbal_;

      rclcpp::TimerBase::SharedPtr manual_vis_timer_, manual_sta_timer_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr manual_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr manual_vp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr manual_mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr manual_uncovered_pub_;
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::Mesh>::SharedPtr mesh_sub_;
      /* Param */
      double suspension_target_ = 0.0;
      bool start_task_ = false;
      double sample_interval_ = 2.0;
      double flight_time_ = 0.0, flight_length_ = 0.0, mean_vel_ = 0.0, coverage_rate_ = 0.0;
      double res_ = 0.0, ground_ = 0.0;
      double vis_inflation_ = 0.0;
      int sample_size_ = 0;
      bool last_rb_pressed_ = false;
      /* Buffer */
      boost::circular_buffer<Eigen::MatrixXd> buffer_mesh_V_;
      boost::circular_buffer<Eigen::MatrixXi> buffer_mesh_F_;
      /* Data */
      vector<Eigen::Vector3d> task_traj_;
      vector<Eigen::VectorXd> key_viewpoints_;
      Eigen::VectorXd last_pose_;
      double cur_interval = 0.0;
      rclcpp::Time flight_start_time_{0, 0, RCL_ROS_TIME};
      Eigen::MatrixXd mesh_V_;
      Eigen::MatrixXi mesh_F_;
      RTCScene cur_scene_ = nullptr;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud_, uncovered_cloud_;
      /* Utils */
      unique_ptr<PerceptionUtils> percep_utils_ = nullptr;
      void triggerManualMode();
      void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy);
      void execCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose, const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& quat);
      void manualVisCallback();
      void meshCallback(const quadrotor_msgs::msg::Mesh& mesh_msg);
      void statisticsCallback();

  };

  // ! --------------------------- Utils ---------------------------
  inline void Manual_Node::triggerManualMode()
  {
    if (this->start_task_ == true) return;

    RCLCPP_ERROR(rclcpp::get_logger("flyco.hierarchical_coverage_planner"), "Received manual mode start trigger.");
    this->start_task_ = true;
    this->flight_start_time_ = node_->now();

    return;
  }

  inline void Manual_Node::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy)
  {
    if (!Joy) return;

    constexpr int JOY_RB_BUTTON = 5;
    bool rb_pressed = Joy->buttons.size() > JOY_RB_BUTTON && Joy->buttons[JOY_RB_BUTTON] == true;
    if (rb_pressed && !this->last_rb_pressed_)
    {
      RCLCPP_WARN(rclcpp::get_logger("flyco.hierarchical_coverage_planner"), "\033[43;37m[FlyCo Joy]\033[47;33m RB pressed: start manual mode task. \033[0m");
      this->triggerManualMode();
    }

    this->last_rb_pressed_ = rb_pressed;

    return;
  }

  inline void Manual_Node::execCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose, const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& quat)
  {
    if (!this->start_task_) return;

    Eigen::Vector3d cur_pos(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z);

    tf2::Quaternion tfQuat_body;
    tf2::fromMsg(pose->pose.pose.orientation, tfQuat_body);
    tf2::Matrix3x3 m_body(tfQuat_body);
    double roll_body, pitch_body, yaw_body;
    m_body.getRPY(roll_body, pitch_body, yaw_body);
    double cur_yaw = yaw_body;

    tf2::Quaternion tfQuat;
    tf2::fromMsg(quat->quaternion, tfQuat);
    tf2::Matrix3x3 m(tfQuat);
    double roll_gimbal, pitch_gimbal, yaw_gimbal;
    m.getRPY(roll_gimbal, pitch_gimbal, yaw_gimbal);
    double cur_pitch = pitch_gimbal;

    // * task trajectory
    if (this->task_traj_.empty())
      this->task_traj_.push_back(cur_pos);
    else
    {
      Eigen::Vector3d last_pos = this->task_traj_.back();
      double dist = (cur_pos - last_pos).norm();
      if (dist > 0.1)
        this->task_traj_.push_back(cur_pos);
    }

    // * key viewpoints
    Eigen::VectorXd cur_viewpoint(5);
    cur_viewpoint << cur_pos(0), cur_pos(1), cur_pos(2), cur_pitch, cur_yaw;
    if (this->key_viewpoints_.empty())
      this->key_viewpoints_.push_back(cur_viewpoint);
    else
    {
      cur_interval += (cur_viewpoint.head(3)-this->last_pose_.head(3)).norm();
      if (cur_interval > this->sample_interval_)
      {
        this->key_viewpoints_.push_back(cur_viewpoint);
        cur_interval = 0.0;
      }
    }

    last_pose_ = cur_viewpoint;

    return;
  }
  
  inline void Manual_Node::manualVisCallback()
  {
    if (!this->start_task_) return;

    if (this->task_traj_.empty()) return;

    nav_msgs::msg::Path traj_msg_;
    traj_msg_.header.stamp = node_->now();
    traj_msg_.header.frame_id = "world";

    for (const auto& point : this->task_traj_)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = point(0);
      pose.pose.position.y = point(1);
      pose.pose.position.z = point(2);

      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 1;

      pose.header.stamp=node_->now();;
      pose.header.frame_id="world";

      traj_msg_.poses.push_back(pose);
    }

    this->manual_traj_pub_->publish(traj_msg_);

    if (this->key_viewpoints_.empty()) return;

    visualization_msgs::msg::Marker vp_mk;
    vp_mk.header.frame_id = "world";
    vp_mk.header.stamp = node_->now();
    vp_mk.id = 0;
    vp_mk.ns = "key_viewpoints";
    vp_mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    vp_mk.color.r = 1.0;
    vp_mk.color.g = 0.0;
    vp_mk.color.b = 0.0;
    vp_mk.color.a = 1.0;
    vp_mk.scale.x = 0.25;
    vp_mk.scale.y = 0.25;
    vp_mk.scale.z = 0.25;
    vp_mk.pose.orientation.w = 1.0;

    vp_mk.action = visualization_msgs::msg::Marker::DELETEALL;
    this->manual_vp_pub_->publish(vp_mk);

    for (const auto& point : this->key_viewpoints_)
    {
      geometry_msgs::msg::Point pt;
      pt.x = point(0);
      pt.y = point(1);
      pt.z = point(2);
      vp_mk.points.push_back(pt);
    }
    
    vp_mk.action = visualization_msgs::msg::Marker::ADD;
    this->manual_vp_pub_->publish(vp_mk);

    if (this->mesh_V_.rows() == 0 || this->mesh_F_.rows() == 0) return;

    visualization_msgs::msg::Marker mesh_marker;
    mesh_marker.header.frame_id = "world";
    mesh_marker.header.stamp = node_->now();
    mesh_marker.ns = "manual_mesh_vis";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    mesh_marker.pose.orientation.w = 1.0;
    mesh_marker.scale.x = 1.0;
    mesh_marker.scale.y = 1.0;
    mesh_marker.scale.z = 1.0;
    mesh_marker.color.a = 0.5;
    mesh_marker.color.r = 0.4;
    mesh_marker.color.g = 0.3;
    mesh_marker.color.b = 0.5;

    mesh_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    manual_mesh_pub_->publish(mesh_marker);

    for (int i = 0; i < this->mesh_F_.rows(); ++i)
    {
        for (int j = 0; j < this->mesh_F_.cols(); ++j)
        {
            int idx = this->mesh_F_(i, j);
            geometry_msgs::msg::Point p;
            p.x = this->mesh_V_(idx, 0);
            p.y = this->mesh_V_(idx, 1);
            p.z = this->mesh_V_(idx, 2);
            mesh_marker.points.push_back(p);
        }
    }

    mesh_marker.action = visualization_msgs::msg::Marker::ADD;
    manual_mesh_pub_->publish(mesh_marker);

    if (this->uncovered_cloud_->points.empty()) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud = *this->uncovered_cloud_;

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "world";
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    this->manual_uncovered_pub_->publish(cloud_msg);

    return;
  }

  inline void Manual_Node::meshCallback(const quadrotor_msgs::msg::Mesh& mesh_msg)
  {
    Eigen::MatrixXd mesh_V;
    Eigen::MatrixXi mesh_F;
    mesh_V.resize(mesh_msg.vertex_num, 3);
    mesh_F.resize(mesh_msg.face_num, 3);

    for (int i=0; i<(int)mesh_msg.vertex_num; ++i)
    {
      mesh_V(i, 0) = mesh_msg.vertex_matrix.data[i*3];
      mesh_V(i, 1) = mesh_msg.vertex_matrix.data[i*3+1];
      mesh_V(i, 2) = mesh_msg.vertex_matrix.data[i*3+2];
    }

    for (int i=0; i<(int)mesh_msg.face_num; ++i)
    {
      mesh_F(i, 0) = mesh_msg.face_matrix.data[i*3];
      mesh_F(i, 1) = mesh_msg.face_matrix.data[i*3+1];
      mesh_F(i, 2) = mesh_msg.face_matrix.data[i*3+2];
    }

    this->buffer_mesh_V_.push_back(mesh_V);
    this->buffer_mesh_F_.push_back(mesh_F);

    return;
  }

  inline void Manual_Node::statisticsCallback()
  {
    if (!this->start_task_) return;
    
    if (this->coverage_rate_ > this->suspension_target_)
    {
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("flyco.hierarchical_coverage_planner"), flyco::steady_logging_clock(), static_cast<int64_t>((1.0) * 1000), "\033[1m\033[30m\033[43m[FlyCo] : ---------------------------- <Flight Finished> ---------------------------- \033[7m\033[30m\033[43m");
    }
    
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("flyco.hierarchical_coverage_planner"), flyco::steady_logging_clock(), static_cast<int64_t>((2.0) * 1000), "\033[1m\033[37m\033[45m[TRAJ_SERVER] Flight Time : %.2f s, Flight Length : %.2f m, Flight Mean Vel : %.2f m/s.\033[7m\033[37m\033[45m", this->flight_time_, this->flight_length_, this->mean_vel_);

    this->flight_time_ = (node_->now() - this->flight_start_time_).seconds();
    this->flight_length_ = 0.0;
    for (int i=1; i<(int)this->task_traj_.size(); ++i)
    {
      this->flight_length_ += (this->task_traj_[i] - this->task_traj_[i-1]).norm();
    }
    this->mean_vel_ = this->flight_length_/this->flight_time_;

    RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("flyco.hierarchical_coverage_planner"), flyco::steady_logging_clock(), static_cast<int64_t>((1.0) * 1000), "\033[1m\033[37m\033[46m[PROGRESS_THREAD] Flight Coverage : \033[7m\033[37m\033[46m" << std::fixed << std::setprecision(2) << this->coverage_rate_ << " %.");

    if (this->cur_scene_ != nullptr)
    {
        rtcReleaseScene(this->cur_scene_);
        this->cur_scene_ = nullptr;
    }
    this->cur_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->uncovered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (this->buffer_mesh_V_.empty() || this->buffer_mesh_F_.empty()) return;
    this->mesh_V_ = this->buffer_mesh_V_.back();
    this->mesh_F_ = this->buffer_mesh_F_.back();

    visibility_st::mesh2scene(this->mesh_V_, this->mesh_F_, this->cur_scene_);
    visibility_st::mesh2pcd(this->mesh_V_, this->mesh_F_, this->res_, this->cur_cloud_);

    if ((int)this->cur_cloud_->points.size() > this->sample_size_)
    {
      pcl::RandomSample<pcl::PointXYZ> rs;
      rs.setInputCloud(this->cur_cloud_);
      rs.setSample(this->sample_size_);
      rs.filter(*this->cur_cloud_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_filter_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i<(int)this->cur_cloud_->points.size(); ++i)
    {
      if (this->cur_cloud_->points[i].z > this->ground_)
        cur_filter_cloud->points.push_back(this->cur_cloud_->points[i]);
    }

    vector<bool> coverage_map((int)cur_filter_cloud->points.size(), false);
    for (int i=0; i<(int)cur_filter_cloud->points.size(); ++i)
    {
      pcl::PointXYZ pt = cur_filter_cloud->points[i];
      for (int j=0; j<(int)this->key_viewpoints_.size(); ++j)
      {
        Eigen::Vector3d vp_pos = this->key_viewpoints_[j].head(3);
        double vp_pitch = this->key_viewpoints_[j](3), vp_yaw = this->key_viewpoints_[j](4);
        Eigen::Vector3d pt_pos(pt.x, pt.y, pt.z);

        this->percep_utils_->setPose_PY(vp_pos, vp_pitch, vp_yaw);
        if (this->percep_utils_->insideFOV(pt_pos) == false)
          continue;
        
        bool visible = visibility_st::visibilityCheck(vp_pos, pt_pos, this->vis_inflation_, this->cur_scene_);
        if (visible == true)
        {
          coverage_map[i] = true;
          break;
        }
      }
    }

    for (int i=0; i<(int)cur_filter_cloud->points.size(); ++i)
    {
      if (coverage_map[i] == false)
        this->uncovered_cloud_->points.push_back(cur_filter_cloud->points[i]);
    }

    int cover_count = count(coverage_map.begin(), coverage_map.end(), true);
    this->coverage_rate_ = 100.0 * (double)cover_count / (double)cur_filter_cloud->points.size();

    return;
  }

} // namespace flyco

#endif
