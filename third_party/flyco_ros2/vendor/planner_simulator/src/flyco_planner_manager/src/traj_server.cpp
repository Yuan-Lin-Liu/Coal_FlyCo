#include <future>
#include <rclcpp/rclcpp.hpp>
#include <flyco_ros2/logging.hpp>
#include <rclcpp/create_timer.hpp>
#include <flyco_ros2/parameters.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
 * Date         :    Jul. 2024
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the trajectory server in FlyCo as the bridge between planner
 *                   and controller.
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

#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "quadrotor_msgs/msg/polynomial_traj_group.hpp"
#include "quadrotor_msgs/msg/eigen_vector_array.hpp"
#include "active_perception/perception_utils.h"
#include "airsim_ros_pkgs/msg/gimbal_angle_euler_cmd.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Eigen>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using std::unique_ptr;
using flyco::PerceptionUtils;

class TrajectoryServer {
  rclcpp::Node::SharedPtr node_;
public:
  explicit TrajectoryServer(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}
private:
/* Param */
bool traj_server_enable_;
double max_vel_;
double fov_h_, fov_w_;
double pitch_upper_, pitch_lower_, drone_radius_;
double cur_interval = 0.0, sample_dist_;
Eigen::VectorXd last_pose = Eigen::VectorXd(5);
bool en_inherit_, en_save_, already_saved_;
double ctrl_interval_;

/* ROS Service */
rclcpp::TimerBase::SharedPtr traj_server_vis_timer_, cmd_timer_, exec_waypts_timer_, robot_vis_timer_, save_timer_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pos_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ori_traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exec_traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_ellipsoid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_vis_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
  rclcpp::Publisher<airsim_ros_pkgs::msg::GimbalAngleEulerCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::EigenVectorArray>::SharedPtr exec_traj_waypts_pub_;
rclcpp::Subscription<quadrotor_msgs::msg::PolynomialTrajGroup>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::EigenVectorArray>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

/* Data */
visualization_msgs::msg::MarkerArray last_ori_marker_set_;
Trajectory<7> last_position_traj_, last_orientation_traj_;
Trajectory<7> newest_position_traj_, newest_orientation_traj_;
double last_dura_ = 0.0, newest_dura_ = 0.0, span_;
rclcpp::Time last_traj_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME), newest_traj_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
int traj_id_ = -2, pub_exec_id_ = 0;
bool received_traj_ = false, end_ = false, statistics_start_ = false;
bool flight_started_ = false, flight_pose_valid_ = false;
vector<Eigen::VectorXd> traj_cmds_, exec_traj_waypts_;
rclcpp::Time flight_start_time_{0, 0, RCL_ROS_TIME};
double flight_length_ = 0.0, flight_time_ = 0.0, flight_mean_vel_ = 0.0;
rclcpp::Time end_time_{0, 0, RCL_ROS_TIME};
vector<Eigen::Vector3d> odom_traj_;
nav_msgs::msg::Odometry cur_odom_;
geometry_msgs::msg::QuaternionStamped cur_gimbal_;
boost::circular_buffer<nav_msgs::msg::Odometry::ConstSharedPtr> buffer_odom_;
boost::circular_buffer<geometry_msgs::msg::QuaternionStamped::ConstSharedPtr> buffer_gimbal_;
boost::circular_buffer<quadrotor_msgs::msg::EigenVectorArray> buffer_path_;

/* Controller */
quadrotor_msgs::msg::PositionCommand cmd_;
airsim_ros_pkgs::msg::GimbalAngleEulerCmd gimbal_cmd_;

/* Odometry */
nav_msgs::msg::Odometry odom_;
geometry_msgs::msg::QuaternionStamped gimbal_orientation_;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, geometry_msgs::msg::QuaternionStamped> SyncPolicyPoseGimbal;
typedef unique_ptr<message_filters::Synchronizer<SyncPolicyPoseGimbal>> SynchronizerPoseGimbal;

unique_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
unique_ptr<message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>> gimbal_sub_;
SynchronizerPoseGimbal sync_odom_gimbal_;

/* Utils */
Eigen::Vector3d jetColorMap(double value)
{
  double r, g, b;
  if (value < 0.0) value = 0.0;
  else if (value > 1.0) value = 1.0;

  if (value < 0.25)
  {
    r = 0.0;
    g = 4.0 * value;
    b = 1.0;
  }
  else if (value < 0.5)
  {
    r = 0.0;
    g = 1.0;
    b = 1.0 - 4.0 * (value - 0.25);
  }
  else if (value < 0.75)
  {
    r = 4.0 * (value - 0.5);
    g = 1.0;
    b = 0.0;
  }
  else
  {
    r = 1.0;
    g = 1.0 - 4.0 * pow((value - 0.75), 1);
    b = 0.0;
  }

  return Eigen::Vector3d(r, g, b);
}

unique_ptr<PerceptionUtils> percepUtils_;

/* Func */
void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg, const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& gimbal)
{
  cur_odom_ = *msg;
  cur_gimbal_ = *gimbal;
  
  if (received_traj_)
  {
    buffer_odom_.push_back(msg);
    buffer_gimbal_.push_back(gimbal);

    Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    odom_traj_.push_back(pos);
  }

  return;
}

void trajCallback(quadrotor_msgs::msg::PolynomialTrajGroup::ConstSharedPtr tMsg)
{
  if (end_) return;

  const quadrotor_msgs::msg::PolynomialTrajGroup &traj = *tMsg;

  if (traj.action == quadrotor_msgs::msg::PolynomialTrajGroup::ACTION_ADD)
  {
    if ((int)traj.trajectory_id > traj_id_)
    {
      traj_id_ = traj.trajectory_id;
      
      Trajectory<7> pos_traj, ori_traj;

      for (auto &piece_pos : traj.pos_traj)
      {
        pos_traj.emplace_back(piece_pos.duration, Eigen::Map<const Eigen::MatrixXd>(&piece_pos.data[0], piece_pos.num_dim, (piece_pos.num_order + 1)));
      }

      for (auto &piece_ori : traj.ori_traj)
      {
        ori_traj.emplace_back(piece_ori.duration, Eigen::Map<const Eigen::MatrixXd>(&piece_ori.data[0], piece_ori.num_dim, (piece_ori.num_order + 1)));
      }

      last_position_traj_.clear();
      last_orientation_traj_.clear();
      last_position_traj_ = newest_position_traj_;
      last_orientation_traj_ = newest_orientation_traj_;
      newest_position_traj_.clear();
      newest_orientation_traj_.clear();
      newest_position_traj_ = pos_traj;
      newest_orientation_traj_ = ori_traj;

      last_traj_start_time_ = newest_traj_start_time_;
      newest_traj_start_time_ = traj.start_time;

      last_dura_ = last_position_traj_.getTotalDuration();
      newest_dura_ = newest_position_traj_.getTotalDuration();
      received_traj_ = true;
      RCLCPP_INFO(rclcpp::get_logger("flyco.flyco_planner_manager"), "[ROS2 timing] received trajectory=%d start=%.9f now=%.9f duration=%.9f", traj_id_, newest_traj_start_time_.seconds(), node_->now().seconds(), newest_dura_);
    }
  }

  return;
}

void pathCallback(const quadrotor_msgs::msg::EigenVectorArray::ConstSharedPtr& pathMsg)
{
  buffer_path_.push_back(*pathMsg);

  return;
}

void finishCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
  if (msg->data == true)
  {
    RCLCPP_INFO(rclcpp::get_logger("flyco.flyco_planner_manager"), "\033[31m[Traj Server] ------------------------------> Stop! \033[32m");
  }

  return;
}

void brakeCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
  if (msg->data == true)
  {
    // * For simulation
    if (end_ == false)
    {
      end_ = true;
      end_time_ = node_->now();
    }
    // * For real DJI
    // DjiFlightController_ExecuteEmergencyBrakeAction();

    RCLCPP_INFO(rclcpp::get_logger("flyco.flyco_planner_manager"), "\033[31m[Traj Server] ------------------------------> Emergency Brake! \033[32m");
  }

  return;
}

void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy)
{
  sensor_msgs::msg::Joy joy_msg;
  if (Joy) joy_msg = *Joy;

  if (joy_msg.buttons[3] == true && !flight_started_)
  {
    flight_started_ = true;
    flight_pose_valid_ = false;
  }

  // Button 'X' to activate statistics
  if (joy_msg.buttons[2] == true) statistics_start_ = true;

  // Button 'down_direction' to save something for inheritance flight
  if (joy_msg.axes[7] < 0.0) en_save_ = true;

  return;
}

void plannedTrajVis()
{
  if (last_position_traj_.getPieceNum() == 0 && newest_position_traj_.getPieceNum() == 0)
    return;

  double pos_T = 0.01;
  double ori_T = 0.3;
  double scale = 3.0;

  // Position
  visualization_msgs::msg::Marker pos_traj_marker;
  pos_traj_marker.header.frame_id = "world";
  pos_traj_marker.header.stamp = node_->now();
  pos_traj_marker.id = 0;
  pos_traj_marker.ns = "planned_pos_traj";
  pos_traj_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  pos_traj_marker.scale.x = 0.10;
  pos_traj_marker.scale.y = 0.10;
  pos_traj_marker.scale.z = 0.10;
  pos_traj_marker.color.a = 1.0;
  pos_traj_marker.pose.orientation.w = 1.0;

  pos_traj_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  pos_traj_pub_->publish(pos_traj_marker);

  // Orientation
  if (!last_ori_marker_set_.markers.empty())
  {
    for (auto &marker : last_ori_marker_set_.markers)
      marker.action = visualization_msgs::msg::Marker::DELETE;
    ori_traj_pub_->publish(last_ori_marker_set_);
  }

  int counter = 0;
  visualization_msgs::msg::MarkerArray ori_traj_marker;

  if (last_position_traj_.getPieceNum() > 0)
  {
    geometry_msgs::msg::Point point;
    // An operator may wait between initialization and scanning. Never visualize
    // extrapolated samples beyond the end of the previous trajectory.
    double t_dura = std::min((newest_traj_start_time_ - last_traj_start_time_).seconds(), last_position_traj_.getTotalDuration());
    for (double t = 0.0; t < t_dura; t += pos_T)
    {
      Eigen::Vector3d X = last_position_traj_.getPos(t);
      double Vel = min(last_position_traj_.getVel(t).norm(), max_vel_)/max_vel_;

      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      Eigen::Vector3d rgb = jetColorMap(Vel);
      pos_traj_marker.color.r = rgb(0);
      pos_traj_marker.color.g = rgb(1);
      pos_traj_marker.color.b = rgb(2);

      pos_traj_marker.points.push_back(point);
      pos_traj_marker.colors.push_back(pos_traj_marker.color);
    }

    for (double t = 0.0; t < t_dura; t += ori_T)
    {
      Eigen::Vector3d X = last_position_traj_.getPos(t);

      double pitchAng = last_orientation_traj_.getPitch(t);
      double yawAng = last_orientation_traj_.getYaw(t);

      if (pitchAng > pitch_upper_ * M_PI / 180.0)  pitchAng = pitch_upper_ * M_PI / 180.0;
      if (pitchAng < pitch_lower_ * M_PI / 180.0)  pitchAng = pitch_lower_ * M_PI / 180.0;

      Eigen::Matrix3d Rwb_y, Rwb_p;
      Rwb_y << cos(yawAng), -sin(yawAng), 0.0, sin(yawAng), cos(yawAng), 0.0, 0.0, 0.0, 1.0;
      Rwb_p << cos(pitchAng), 0.0, -sin(pitchAng), 0.0, 1.0, 0.0, sin(pitchAng), 0.0, cos(pitchAng);

      Eigen::Vector3d X_orientation = X + Rwb_y*Rwb_p*Eigen::Vector3d(scale, 0.0, 0.0);

      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = node_->now();
      mk.id = counter;
      mk.ns = "planned_ori_traj";
      mk.type = visualization_msgs::msg::Marker::ARROW;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.2;
      mk.scale.y = 0.4;
      mk.scale.z = 0.3;
      mk.color.r = 0.5;
      mk.color.g = 0.5;
      mk.color.b = 0.4;
      mk.color.a = 1.0;

      geometry_msgs::msg::Point pt_;
      pt_.x = X(0);
      pt_.y = X(1);
      pt_.z = X(2);
      mk.points.push_back(pt_);

      pt_.x = X_orientation(0);
      pt_.y = X_orientation(1);
      pt_.z = X_orientation(2);
      mk.points.push_back(pt_);

      ori_traj_marker.markers.push_back(mk);
      counter++;
    }
  }

  if (newest_position_traj_.getPieceNum() > 0)
  {
    geometry_msgs::msg::Point point;
    for (double t = 0.0; t < newest_dura_; t += pos_T)
    {
      Eigen::Vector3d X = newest_position_traj_.getPos(t);
      double Vel = min(newest_position_traj_.getVel(t).norm(), max_vel_)/max_vel_;

      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      Eigen::Vector3d rgb = jetColorMap(Vel);
      pos_traj_marker.color.r = rgb(0);
      pos_traj_marker.color.g = rgb(1);
      pos_traj_marker.color.b = rgb(2);

      pos_traj_marker.points.push_back(point);
      pos_traj_marker.colors.push_back(pos_traj_marker.color);
    }

    for (double t = 0.0; t < newest_dura_; t += ori_T)
    {
      Eigen::Vector3d X = newest_position_traj_.getPos(t);

      double pitchAng = newest_orientation_traj_.getPitch(t);
      double yawAng = newest_orientation_traj_.getYaw(t);

      if (pitchAng > pitch_upper_ * M_PI / 180.0)  pitchAng = pitch_upper_ * M_PI / 180.0;
      if (pitchAng < pitch_lower_ * M_PI / 180.0)  pitchAng = pitch_lower_ * M_PI / 180.0;

      Eigen::Matrix3d Rwb_y, Rwb_p;
      Rwb_y << cos(yawAng), -sin(yawAng), 0.0, sin(yawAng), cos(yawAng), 0.0, 0.0, 0.0, 1.0;
      Rwb_p << cos(pitchAng), 0.0, -sin(pitchAng), 0.0, 1.0, 0.0, sin(pitchAng), 0.0, cos(pitchAng);

      Eigen::Vector3d X_orientation = X + Rwb_y*Rwb_p*Eigen::Vector3d(scale, 0.0, 0.0);

      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "world";
      mk.header.stamp = node_->now();
      mk.id = counter;
      mk.ns = "planned_ori_traj";
      mk.type = visualization_msgs::msg::Marker::ARROW;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.2;
      mk.scale.y = 0.4;
      mk.scale.z = 0.3;
      mk.color.r = 0.5;
      mk.color.g = 0.5;
      mk.color.b = 0.4;
      mk.color.a = 1.0;

      geometry_msgs::msg::Point pt_;
      pt_.x = X(0);
      pt_.y = X(1);
      pt_.z = X(2);
      mk.points.push_back(pt_);

      pt_.x = X_orientation(0);
      pt_.y = X_orientation(1);
      pt_.z = X_orientation(2);
      mk.points.push_back(pt_);

      ori_traj_marker.markers.push_back(mk);
      counter++;
    }
  }

  pos_traj_marker.action = visualization_msgs::msg::Marker::ADD;
  pos_traj_pub_->publish(pos_traj_marker);

  ori_traj_pub_->publish(ori_traj_marker);
  last_ori_marker_set_ = ori_traj_marker;

  return;
}

void execTrajVis(vector<Eigen::VectorXd> cmd_path, vector<Eigen::Vector3d> odom_path)
{
  if (cmd_path.empty())
    return;

  nav_msgs::msg::Path traj_msg_;
  traj_msg_.header.stamp = node_->now();
  traj_msg_.header.frame_id = "world";

  for (const auto& point : cmd_path)
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

  exec_traj_pub_->publish(traj_msg_);

  if (odom_path.empty())
    return;

  nav_msgs::msg::Path odom_msg_;
  odom_msg_.header.stamp = node_->now();
  odom_msg_.header.frame_id = "world";

  for (const auto& point : odom_path)
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

    odom_msg_.poses.push_back(pose);
  }

  odom_traj_pub_->publish(odom_msg_);

  return;
}

void visCallback()
{
  plannedTrajVis();
  execTrajVis(traj_cmds_, odom_traj_);

  pub_exec_id_++;
  
  return;
}

void publishGimbalCmd(const airsim_ros_pkgs::msg::GimbalAngleEulerCmd& gimbal_cmd)
{
  gimbal_cmd_pub_->publish(gimbal_cmd);
}

void delayedPublishThread(const airsim_ros_pkgs::msg::GimbalAngleEulerCmd& gimbal_cmd)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(0)); // ! AirSim Trick delay 500ms
  publishGimbalCmd(gimbal_cmd);
}

void cmdCallback()
{
  if (en_save_) return;

  if (!received_traj_)
    return;

  if (statistics_start_) RCLCPP_INFO_THROTTLE(rclcpp::get_logger("flyco.flyco_planner_manager"), flyco::steady_logging_clock(), static_cast<int64_t>((2.0) * 1000), "\033[1m\033[37m\033[45m[TRAJ_SERVER] Flight Time : %.2f s, Flight Length : %.2f m, Flight Mean Vel : %.2f m/s.\033[7m\033[37m\033[45m", flight_time_, flight_length_, flight_mean_vel_);

  if (end_)
  {
    return; // back to hover
  }
  
  rclcpp::Time cur_time = node_->now();
  double t_old = (cur_time - last_traj_start_time_).seconds();
  double t_new = (cur_time - newest_traj_start_time_).seconds();

  Trajectory<7> cur_pos_traj, cur_ori_traj;
  double duration = 0.0;

  if (t_old > 0 && t_new < 0 && last_position_traj_.getPieceNum() > 0)
  {
    cur_pos_traj = last_position_traj_;
    cur_ori_traj = last_orientation_traj_;
    duration = last_dura_;
    span_ = t_old;
  }
  else if (t_new >= 0 && newest_position_traj_.getPieceNum() > 0)
  {
    cur_pos_traj = newest_position_traj_;
    cur_ori_traj = newest_orientation_traj_;
    duration = newest_dura_;
    span_ = t_new;
  }
  else
  {
    return;
  }

  if (span_ > duration)
  {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("flyco.flyco_planner_manager"), flyco::steady_logging_clock(), static_cast<int64_t>((5.0) * 1000), "[ROS2 timing] expired trajectory=%d span=%.9f duration=%.9f now=%.9f start=%.9f", traj_id_, span_, duration, cur_time.seconds(), newest_traj_start_time_.seconds());
    flight_pose_valid_ = false;
    return; // back to hover
  }

  Eigen::Vector3d pos, vel, acc, jer;
  double pitch, yaw, yaw_vel;

  pos = cur_pos_traj.getPos(span_);
  vel = cur_pos_traj.getVel(span_);
  acc = cur_pos_traj.getAcc(span_);
  jer = cur_pos_traj.getJer(span_);
  pitch = cur_ori_traj.getPitch(span_);
  yaw = cur_ori_traj.getYaw(span_);
  yaw_vel = cur_ori_traj.getYawd(span_);

  if (pitch > pitch_upper_ * M_PI / 180.0)  pitch = pitch_upper_ * M_PI / 180.0;
  if (pitch < pitch_lower_ * M_PI / 180.0)  pitch = pitch_lower_ * M_PI / 180.0;

  Eigen::VectorXd pose_cmd(5);
  pose_cmd << pos(0), pos(1), pos(2), pitch, yaw;

  // X activates task statistics after the init path.
  if (flight_started_ && statistics_start_)
  {
    if (flight_pose_valid_)
    {
      flight_time_ += ctrl_interval_;
      flight_length_ += (pose_cmd.head(3)-last_pose.head(3)).norm();
      flight_mean_vel_ = flight_time_ > 0.0 ? flight_length_/flight_time_ : 0.0;
    }
    flight_pose_valid_ = true;
  }

  // Keep the init trajectory out of the planner's executed-waypoint history.
  if (statistics_start_)
  {
    traj_cmds_.push_back(pose_cmd);

    // add to exec_traj_waypts_
    if (exec_traj_waypts_.empty())
    {
      exec_traj_waypts_.push_back(pose_cmd);
    }
    else
    {
      cur_interval += (pose_cmd.head(3)-last_pose.head(3)).norm();
      if (cur_interval > sample_dist_)
      {
        exec_traj_waypts_.push_back(pose_cmd);
        cur_interval = 0.0;
      }
    }
  }

  last_pose = pose_cmd;

  cmd_.header.stamp = cur_time;
  cmd_.position.x = pos(0);
  cmd_.position.y = pos(1);
  cmd_.position.z = pos(2);
  cmd_.velocity.x = vel(0);
  cmd_.velocity.y = vel(1);
  cmd_.velocity.z = vel(2);
  cmd_.acceleration.x = acc(0);
  cmd_.acceleration.y = acc(1);
  cmd_.acceleration.z = acc(2);
  cmd_.jerk.x = jer(0);
  cmd_.jerk.y = jer(1);
  cmd_.jerk.z = jer(2);
  cmd_.yaw = yaw;
  cmd_.yaw_dot = yaw_vel;
  // gimbal pitch
  gimbal_cmd_.header.stamp = cur_time;
  gimbal_cmd_.roll = 0.0;
  gimbal_cmd_.pitch = pitch*180.0/M_PI;
  gimbal_cmd_.yaw = 0.0;

  gimbal_tasks_.erase(std::remove_if(gimbal_tasks_.begin(), gimbal_tasks_.end(),
    [](std::future<void>& task) {
      if (task.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return false;
      task.get(); return true;
    }), gimbal_tasks_.end());
  gimbal_tasks_.emplace_back(std::async(std::launch::async,
    [publisher = gimbal_cmd_pub_, message = gimbal_cmd_]() {
      if (!rclcpp::ok()) return;
      try { publisher->publish(message); }
      catch (const rclcpp::exceptions::RCLError&) { if (rclcpp::ok()) throw; }
    }));
  pos_cmd_pub_->publish(cmd_);
}

void execTrajWayptsCallback()
{
  if (exec_traj_waypts_.empty())
    return;

  quadrotor_msgs::msg::EigenVectorArray waypts_msg;
  waypts_msg.header.stamp = node_->now();

  for (const auto& vec : exec_traj_waypts_)
  {
    std_msgs::msg::Float64MultiArray array;
    for (int i = 0; i < (int)vec.size(); ++i)
    {
      array.data.push_back(vec[i]);
    }
    waypts_msg.vectors.push_back(array);
  }

  exec_traj_waypts_pub_->publish(waypts_msg);

  return;
}

void robotVisCallback()
{
  if (en_save_) return;

  if (!received_traj_)
    return;

  rclcpp::Time cur_time = end_ == true? end_time_ : node_->now();
  double t_old = (cur_time - last_traj_start_time_).seconds();
  double t_new = (cur_time - newest_traj_start_time_).seconds();

  Trajectory<7> cur_pos_traj, cur_ori_traj;
  double duration = 0.0, t_cur = 0.0;

  if (t_old > 0 && t_new < 0 && last_position_traj_.getPieceNum() > 0)
  {
    cur_pos_traj = last_position_traj_;
    cur_ori_traj = last_orientation_traj_;
    duration = last_dura_;
    t_cur = t_old;
  }
  else if (t_new >= 0 && newest_position_traj_.getPieceNum() > 0)
  {
    cur_pos_traj = newest_position_traj_;
    cur_ori_traj = newest_orientation_traj_;
    duration = newest_dura_;
    t_cur = t_new;
  }
  else
  {
    return;
  }

  if (t_cur > duration)
  {
    if (t_cur > 1e6)
      return;
    else
      t_cur = duration;
  }

  Eigen::Vector3d pos;
  double pitch, yaw;
  pos = cur_pos_traj.getPos(t_cur);
  pitch = cur_ori_traj.getPitch(t_cur);
  yaw = cur_ori_traj.getYaw(t_cur);

  if (pitch > pitch_upper_ * M_PI / 180.0)  pitch = pitch_upper_ * M_PI / 180.0;
  if (pitch < pitch_lower_ * M_PI / 180.0)  pitch = pitch_lower_ * M_PI / 180.0;

  // * Robot
  visualization_msgs::msg::Marker cmd_vis;
  cmd_vis.header.frame_id = "world";
  cmd_vis.header.stamp = node_->now();
  cmd_vis.id = 0;
  cmd_vis.ns = "current_robot_vis";
  cmd_vis.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  cmd_vis.color.r = 0.0;
  cmd_vis.color.g = 0.0;
  cmd_vis.color.b = 0.0;
  cmd_vis.color.a = 1.0;
  cmd_vis.scale.x = 3*drone_radius_;
  cmd_vis.scale.y = 3*drone_radius_;
  cmd_vis.scale.z = 3*drone_radius_;
  // cmd_vis.mesh_resource = "package://vis_utils/DJI_M30T_S.dae";
  cmd_vis.mesh_resource = "package://flyco_ros2/vendor/planner_simulator/src/vis_utils/f250.dae";

  // ADD updates the existing namespace/id without hiding the mesh between frames.
  double yaw_drone = yaw;
  double roll_odom = 0.0, pitch_odom = 0.0, yaw_odom = 0.0;
  tf2::Quaternion q_odom;
  tf2::fromMsg(cur_odom_.pose.pose.orientation, q_odom);
  tf2::Matrix3x3(q_odom).getRPY(roll_odom, pitch_odom, yaw_odom);

  tf2::Quaternion q_vis;
  q_vis.setRPY(roll_odom, pitch_odom, yaw_drone);
  q_vis.normalize();

  cmd_vis.pose.orientation.x = q_vis.x();
  cmd_vis.pose.orientation.y = q_vis.y();
  cmd_vis.pose.orientation.z = q_vis.z();
  cmd_vis.pose.orientation.w = q_vis.w();
  cmd_vis.pose.position.x = pos(0);
  cmd_vis.pose.position.y = pos(1);
  cmd_vis.pose.position.z = pos(2);
  cmd_vis.action = visualization_msgs::msg::Marker::ADD;
  robot_vis_pub_->publish(cmd_vis);

  // * Robot ellipsoid
  visualization_msgs::msg::Marker ellipsoid;
  ellipsoid.header.frame_id = "world";
  ellipsoid.header.stamp = node_->now();
  ellipsoid.id = 0;
  ellipsoid.ns = "current_robot_ellipsoid";
  ellipsoid.type = visualization_msgs::msg::Marker::SPHERE;
  ellipsoid.color.r = 0.0;
  ellipsoid.color.g = 1.0;
  ellipsoid.color.b = 0.6;
  ellipsoid.color.a = 0.4;
  ellipsoid.scale.x = 3*drone_radius_;
  ellipsoid.scale.y = 3*drone_radius_;
  ellipsoid.scale.z = drone_radius_;

  ellipsoid.pose.orientation.w = 1.0;
  ellipsoid.pose.position.x = pos(0);
  ellipsoid.pose.position.y = pos(1);
  ellipsoid.pose.position.z = pos(2);
  ellipsoid.action = visualization_msgs::msg::Marker::ADD;
  robot_ellipsoid_pub_->publish(ellipsoid);

  // * FoV
  double fov_line_scale = 0.1;
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = node_->now();
  mk.id = 0;
  mk.ns = "current_FOV";
  mk.type = visualization_msgs::msg::Marker::LINE_LIST;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  
  mk.color.a = 1.0;
  mk.scale.x = fov_line_scale;
  mk.scale.y = fov_line_scale;
  mk.scale.z = fov_line_scale;
  mk.pose.orientation.w = 1.0;

  percepUtils_->setPose_PY(pos, pitch, yaw);
  vector<Eigen::Vector3d> list1, list2;
  percepUtils_->getFOV_PY(list1, list2);

  if (list1.empty())
  {
    mk.action = visualization_msgs::msg::Marker::DELETE;
    fov_vis_pub_->publish(mk);
    return;
  }

  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) 
  {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::msg::Marker::ADD;
  fov_vis_pub_->publish(mk);
}

void saveCallback()
{
  if (!en_save_ || already_saved_) return;

  already_saved_ = true;

  string data_path = (ament_index_cpp::get_package_share_directory("flyco_ros2") + "/vendor/planner_simulator/src/flyco_planner_manager") + "/inherit_data";
  string inherit_point_file = data_path + "/inherit_point.yaml";
  string exec_traj_file = data_path + "/exec_traj.yaml";
  string exec_waypts_file = data_path + "/exec_waypts.yaml";
  string local_path_file = data_path + "/local_path.yaml";
  string statistics_file = data_path + "/statistics.yaml";

  // * inherit point
  double p_x = cur_odom_.pose.pose.position.x;
  double p_y = cur_odom_.pose.pose.position.y;
  double p_z = cur_odom_.pose.pose.position.z;

  tf2::Quaternion tfQuat_body;
  tf2::fromMsg(cur_odom_.pose.pose.orientation, tfQuat_body);
  tf2::Matrix3x3 m_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  m_body.getRPY(roll_body, pitch_body, yaw_body);

  tf2::Quaternion tfQuat;
  tf2::fromMsg(cur_gimbal_.quaternion, tfQuat);
  tf2::Matrix3x3 m(tfQuat);
  double roll_gimbal, pitch_gimbal, yaw_gimbal;
  m.getRPY(roll_gimbal, pitch_gimbal, yaw_gimbal);

  double pitch = pitch_gimbal * 180.0 / M_PI;
  double yaw = yaw_body * 180.0 / M_PI;

  vector<double> inherit_point = {p_x, p_y, p_z, pitch, yaw};

  vector<vector<double>> ip_vector = {inherit_point};

  YAML::Emitter ip_emitter;
    
  ip_emitter << YAML::BeginMap;
  ip_emitter << YAML::Key << "waypoints";
  ip_emitter << YAML::Value << YAML::BeginSeq;
    
  for (const auto& wp : ip_vector) 
  {
    ip_emitter << YAML::Flow;             
    ip_emitter << YAML::BeginSeq;
    for (double coord : wp) 
    {
      ip_emitter << coord;
    }
    ip_emitter << YAML::EndSeq;
  }
  
  ip_emitter << YAML::EndSeq;

  ip_emitter << YAML::Key << "velocity";
  ip_emitter << YAML::Value << YAML::BeginSeq;
  ip_emitter << YAML::Flow; 
  ip_emitter << YAML::BeginSeq << cur_odom_.twist.twist.linear.x << cur_odom_.twist.twist.linear.y << cur_odom_.twist.twist.linear.z << YAML::EndSeq;
  ip_emitter << YAML::EndSeq;

  ip_emitter << YAML::EndMap;

  ofstream ipf_out(inherit_point_file, ios::trunc);
  ipf_out << ip_emitter.c_str();
  ipf_out.close();

  // * exec traj
  if (!traj_cmds_.empty())
  {
    vector<Eigen::VectorXd> sampled_traj_cmds = {traj_cmds_.front()};
    for (auto p : traj_cmds_)
    {
      if ((p.head(3)-sampled_traj_cmds.back().head(3)).norm() > 0.1)
      {
        sampled_traj_cmds.push_back(p);
      }
    }

    YAML::Emitter et_emitter;
    
    et_emitter << YAML::BeginMap;
    et_emitter << YAML::Key << "exec_traj";
    et_emitter << YAML::Value << YAML::BeginSeq;
    
    for (const auto& wp : sampled_traj_cmds) 
    {
      et_emitter << YAML::Flow;             
      et_emitter << YAML::BeginSeq;
      for (Eigen::Index i = 0; i < wp.size(); ++i)
      {
        et_emitter << wp[i];
      }
      et_emitter << YAML::EndSeq;
    }
    
    et_emitter << YAML::EndSeq;
    et_emitter << YAML::EndMap;

    ofstream etf_out(exec_traj_file, ios::trunc);
    etf_out << et_emitter.c_str();
    etf_out.close();
  }

  // * exec waypts
  if (!exec_traj_waypts_.empty())
  {
    YAML::Emitter ew_emitter;
    
    ew_emitter << YAML::BeginMap;
    ew_emitter << YAML::Key << "exec_waypts";
    ew_emitter << YAML::Value << YAML::BeginSeq;
    
    for (const auto& wp : exec_traj_waypts_) 
    {
      ew_emitter << YAML::Flow;             
      ew_emitter << YAML::BeginSeq;
      for (Eigen::Index i = 0; i < wp.size(); ++i)
      {
        ew_emitter << wp[i];
      }
      ew_emitter << YAML::EndSeq;
    }
    
    ew_emitter << YAML::EndSeq;
    ew_emitter << YAML::EndMap;

    ofstream ewf_out(exec_waypts_file, ios::trunc);
    ewf_out << ew_emitter.c_str();
    ewf_out.close();
  }

  // * local path
  if (!buffer_path_.empty())
  {
    vector<Eigen::VectorXd> local_newest_path;
    for (const auto& array : buffer_path_.back().vectors)
    {
      Eigen::VectorXd vec(array.data.size());
      for (size_t i = 0; i < array.data.size(); ++i)
      {
        vec[i] = array.data[i];
      }
      local_newest_path.push_back(vec);
    }

    YAML::Emitter lp_emitter;

    lp_emitter << YAML::BeginMap;
    lp_emitter << YAML::Key << "local_path";
    lp_emitter << YAML::Value << YAML::BeginSeq;

    for (const auto& wp : local_newest_path) 
    {
      lp_emitter << YAML::Flow;             
      lp_emitter << YAML::BeginSeq;
      for (Eigen::Index i = 0; i < wp.size(); ++i)
      {
        lp_emitter << wp[i];
      }
      lp_emitter << YAML::EndSeq;
    }

    lp_emitter << YAML::EndSeq;
    lp_emitter << YAML::EndMap;

    ofstream lpf_out(local_path_file, ios::trunc);
    lpf_out << lp_emitter.c_str();
    lpf_out.close();
  }

  // * statistics
  YAML::Node statistics_node;
  statistics_node["flight_time"] = flight_time_;
  statistics_node["flight_length"] = flight_length_;

  ofstream sf_out(statistics_file, ios::trunc);
  sf_out << statistics_node;
  sf_out.close();

  RCLCPP_ERROR(rclcpp::get_logger("flyco.flyco_planner_manager"), "traj server inheritance flight saved!");

  return;
}

void loadInheritData()
{
  string data_path = (ament_index_cpp::get_package_share_directory("flyco_ros2") + "/vendor/planner_simulator/src/flyco_planner_manager") + "/inherit_data";
  string exec_traj_file = data_path + "/exec_traj.yaml";
  string exec_waypts_file = data_path + "/exec_waypts.yaml";
  string statistics_file = data_path + "/statistics.yaml";

  // * read exec traj
  traj_cmds_.clear();

  YAML::Node exec_traj_node = YAML::LoadFile(exec_traj_file);
  for (const auto& point : exec_traj_node["exec_traj"])
  {
    Eigen::VectorXd ep(5);
    ep << point[0].as<double>(), point[1].as<double>(), point[2].as<double>(), point[3].as<double>(), point[4].as<double>();
    traj_cmds_.push_back(ep);
  }

  // * read exec waypts
  exec_traj_waypts_.clear();

  YAML::Node exec_waypts_node = YAML::LoadFile(exec_waypts_file);
  for (const auto& point : exec_waypts_node["exec_waypts"])
  {
    Eigen::VectorXd ep(5);
    ep << point[0].as<double>(), point[1].as<double>(), point[2].as<double>(), point[3].as<double>(), point[4].as<double>();
    exec_traj_waypts_.push_back(ep);
  }

  // * read statistics
  YAML::Node statistics_node = YAML::LoadFile(statistics_file);
  flight_time_ = statistics_node["flight_time"].as<double>();
  flight_length_ = statistics_node["flight_length"].as<double>();

  RCLCPP_INFO(rclcpp::get_logger("flyco.flyco_planner_manager"), "\033[33m[Traj Server] Inheritance Flight Data is Imported! \033[32m");

  return;
}


std::vector<std::future<void>> gimbal_tasks_;

public:
  int run() {

  
  const auto& nh = node_;

  traj_server_enable_ = true;

  if (!traj_server_enable_)
  {
    RCLCPP_INFO(rclcpp::get_logger("flyco.flyco_planner_manager"), "\033[33m[Traj Server] ------------------------------> Disabled! \033[32m");
    rclcpp::spin(node_);
    return 0;
  }

  // Statistics initialization
  flight_time_ = 0.0;
  flight_length_ = 0.0;
  flight_mean_vel_ = 0.0;

  max_vel_ = flyco::parameter_or(nh, "traj_server.max_vel", -1.0);
  fov_h_ = flyco::parameter_or(nh, "traj_server.fov_h", -1.0);
  fov_w_ = flyco::parameter_or(nh, "traj_server.fov_w", -1.0);
  pitch_upper_ = flyco::parameter_or(nh, "traj_server.pitch_upper", -1.0);
  pitch_lower_ = flyco::parameter_or(nh, "traj_server.pitch_lower", -1.0);
  drone_radius_ = flyco::parameter_or(nh, "traj_server.radius", 0.3);
  en_inherit_ = flyco::parameter_or(nh, "traj_server.enable_inherit", false);
  en_save_ = false;
  already_saved_ = false;
  ctrl_interval_ = 0.01;

  if (en_inherit_) loadInheritData();

  percepUtils_.reset(new PerceptionUtils);
  percepUtils_->init(nh);

  // Buffer
  int num = 20;
  buffer_odom_ = boost::circular_buffer<nav_msgs::msg::Odometry::ConstSharedPtr>(num);
  buffer_gimbal_ = boost::circular_buffer<geometry_msgs::msg::QuaternionStamped::ConstSharedPtr>(num);
  buffer_path_ = boost::circular_buffer<quadrotor_msgs::msg::EigenVectorArray>(num);

  // Control parameter
  cmd_.kx = { 5.7, 5.7, 6.2 };
  cmd_.kv = { 3.4, 3.4, 4.0 };
  cmd_.header.stamp = node_->now();
  cmd_.header.frame_id = "world";
  cmd_.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_.trajectory_id = traj_id_;
  cmd_.velocity.x = 0.0;
  cmd_.velocity.y = 0.0;
  cmd_.velocity.z = 0.0;
  cmd_.acceleration.x = 0.0;
  cmd_.acceleration.y = 0.0;
  cmd_.acceleration.z = 0.0;
  cmd_.jerk.x = 0.0;
  cmd_.jerk.y = 0.0;
  cmd_.jerk.z = 0.0;
  cmd_.yaw = 0.0;
  cmd_.yaw_dot = 0.0;
  // gimbal
  gimbal_cmd_.header.stamp = node_->now();
  gimbal_cmd_.camera_name = "front_center";
  gimbal_cmd_.vehicle_name = "drone_1";
  gimbal_cmd_.roll = 0.0;
  gimbal_cmd_.pitch = 0.0;
  gimbal_cmd_.yaw = 0.0;

  /* Odometry */
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh, "/traj_server/odom", rclcpp::QoS(50).reliable().get_rmw_qos_profile()));
  gimbal_sub_.reset(new message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>(nh, "/traj_server/gimbal", rclcpp::QoS(50).reliable().get_rmw_qos_profile()));
  sync_odom_gimbal_.reset(new message_filters::Synchronizer<SyncPolicyPoseGimbal>(SyncPolicyPoseGimbal(50), *odom_sub_, *gimbal_sub_));
  sync_odom_gimbal_->registerCallback(std::bind(&TrajectoryServer::odomCallback, this, std::placeholders::_1, std::placeholders::_2));

  last_traj_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  newest_traj_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  sample_dist_ = 1.0;

  pos_traj_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("/traj_server/planned_pos_traj", rclcpp::QoS(1).reliable());
  ori_traj_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/traj_server/planned_ori_traj", rclcpp::QoS(1).reliable());
  exec_traj_pub_ = nh->create_publisher<nav_msgs::msg::Path>("/traj_server/exec_traj", rclcpp::QoS(1).reliable().transient_local());
  odom_traj_pub_ = nh->create_publisher<nav_msgs::msg::Path>("/traj_server/odom_traj", rclcpp::QoS(1).reliable().transient_local());
  exec_traj_waypts_pub_ = nh->create_publisher<quadrotor_msgs::msg::EigenVectorArray>("/traj_server/exec_traj_waypts", rclcpp::QoS(1).reliable());
  pos_cmd_pub_ = nh->create_publisher<quadrotor_msgs::msg::PositionCommand>("/traj_server/pos_cmd", rclcpp::QoS(50).reliable());
  gimbal_cmd_pub_ = nh->create_publisher<airsim_ros_pkgs::msg::GimbalAngleEulerCmd>("/traj_server/gimbal_cmd", rclcpp::QoS(50).reliable());
  robot_vis_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("/traj_server/robot_vis", rclcpp::QoS(1).reliable());
  robot_ellipsoid_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("/traj_server/robot_ellipsoid", rclcpp::QoS(1).reliable());
  fov_vis_pub_ = nh->create_publisher<visualization_msgs::msg::Marker>("/traj_server/fov_vis", rclcpp::QoS(1).reliable());

  traj_server_vis_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&TrajectoryServer::visCallback, this));
  cmd_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(ctrl_interval_), std::bind(&TrajectoryServer::cmdCallback, this));
  exec_waypts_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&TrajectoryServer::execTrajWayptsCallback, this));
  robot_vis_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&TrajectoryServer::robotVisCallback, this));
  save_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&TrajectoryServer::saveCallback, this));

  traj_sub_ = nh->create_subscription<quadrotor_msgs::msg::PolynomialTrajGroup>("/local_newest_traj", rclcpp::QoS(1).reliable(), std::bind(&TrajectoryServer::trajCallback, this, std::placeholders::_1));
  finish_sub_ = nh->create_subscription<std_msgs::msg::Bool>("/flight_end", rclcpp::QoS(1).reliable(), std::bind(&TrajectoryServer::finishCallback, this, std::placeholders::_1));
  brake_sub_ = nh->create_subscription<std_msgs::msg::Bool>("/emergency_brake", rclcpp::QoS(1).reliable(), std::bind(&TrajectoryServer::brakeCallback, this, std::placeholders::_1));
  path_sub_ = nh->create_subscription<quadrotor_msgs::msg::EigenVectorArray>("/local_newest_path", rclcpp::QoS(1).reliable(), std::bind(&TrajectoryServer::pathCallback, this, std::placeholders::_1));
  joy_sub_ = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::QoS(50).reliable(), std::bind(&TrajectoryServer::joyCallback, this, std::placeholders::_1));

  rclcpp::sleep_for(std::chrono::nanoseconds(rclcpp::Duration::from_seconds(1.0).nanoseconds()));

  RCLCPP_INFO(rclcpp::get_logger("flyco.flyco_planner_manager"), "\033[33m[Traj Server] ------------------------------> Ready! \033[32m");
  rclcpp::spin(node_);
  
  return 0;

  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("traj_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  int result;
  {
    TrajectoryServer application(node);
    result = application.run();
  }
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
