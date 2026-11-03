/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Feb. 2025
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the header file of Init_Traj class, which implements the
 *                   generation of initial trajectory in FlyCo.
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

#ifndef _INIT_TRAJ_H_
#define _INIT_TRAJ_H_

#include "plan_env/sdf_map.h"
#include "hierarchical_coverage_planner/hctraj.h"
#include "plan_utils/path_tools.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/trajectory.hpp"
#include "quadrotor_msgs/msg/polynomial_traj_group.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Core>
#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using std::shared_ptr;
using std::unique_ptr;

namespace flyco
{

class SDFMap;
class TrajGenerator;

class Init_Traj
{
  rclcpp::Node::SharedPtr node_;

public:
  Init_Traj();
  ~Init_Traj();
  /* Func */
  void init(const rclcpp::Node::SharedPtr& nh);
  void setMap(shared_ptr<SDFMap>& map);

private:
  /* Utils */
  shared_ptr<SDFMap> mapping_ = nullptr;
  unique_ptr<TrajGenerator> traj_generator_ = nullptr;
  /* Param */
  bool start_gen_ = false;
  bool finish_init_ = false;
  bool en_inherit_ = false;
  int traj_id_ = -1;
  int path_size_ = 0;
  int num_path_ = 0;
  string init_path_file_;
  /* Func */
  void run();
  void readInitialPoints();
  void trajConverter(const Trajectory<7> &pos, const Trajectory<7> &ori, quadrotor_msgs::msg::PolynomialTrajGroup &msg, const rclcpp::Time &cur_stamp, int &traj_id);
  void runCallback();
  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy);
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void gimbalCallback(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg);
  /* ROS Service */
  rclcpp::TimerBase::SharedPtr task_timer_;
  rclcpp::Publisher<quadrotor_msgs::msg::PolynomialTrajGroup>::SharedPtr traj_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr gimbal_sub_;
  /* Data */
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::QuaternionStamped gimbal_;
  vector<Eigen::VectorXd> waypoints_;
  vector<bool> indicators_;
  vector<vector<Eigen::VectorXd>> paths_;
  vector<vector<bool>> paths_indicators_;
};

}

#endif
