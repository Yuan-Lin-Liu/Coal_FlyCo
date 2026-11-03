#include <rclcpp/rclcpp.hpp>
#include <flyco_ros2/logging.hpp>
#include <rclcpp/create_timer.hpp>
#include <flyco_ros2/parameters.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/polynomial_matrix.hpp>
#include <quadrotor_msgs/msg/polynomial_traj_group.hpp>
#include <sensor_msgs/msg/joy.hpp>
/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Feb. 2025
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the main algorithm of initial trajectory generation in FlyCo.
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

#include "hierarchical_coverage_planner/init_traj.h"
#include <cmath>

namespace flyco
{
  Init_Traj::Init_Traj(){
  }

  Init_Traj::~Init_Traj(){
  }

  void Init_Traj::init(const rclcpp::Node::SharedPtr& nh)
  {
  node_ = nh;
    // * Params Initialization
    this->en_inherit_ = flyco::parameter_or(nh, "flyco_planner.enable_inherit", false);
    this->init_path_file_ = flyco::parameter_or(nh, "flyco_planner.init_path_file", std::string(""));
    this->readInitialPoints();
    this->start_gen_ = false;
    this->finish_init_ = false;
    this->traj_id_ = -this->path_size_ + 1;

    // * Module Initialization
    this->traj_generator_.reset(new TrajGenerator);
    this->traj_generator_->init(nh);

    // * ROS Initialization
    this->task_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&Init_Traj::runCallback, this));
    this->traj_pub_ = nh->create_publisher<quadrotor_msgs::msg::PolynomialTrajGroup>("/local_newest_traj", rclcpp::QoS(1).reliable());
    this->joy_sub_ = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::QoS(50).reliable(), std::bind(&Init_Traj::joyCallback, this, std::placeholders::_1));
    this->odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>("/flyco_planner/body_pose", rclcpp::QoS(1).reliable(), std::bind(&Init_Traj::odomCallback, this, std::placeholders::_1));
    this->gimbal_sub_ = nh->create_subscription<geometry_msgs::msg::QuaternionStamped>("/flyco_planner/gimbal_pose", rclcpp::QoS(1).reliable(), std::bind(&Init_Traj::gimbalCallback, this, std::placeholders::_1));

    return;
  }

  void Init_Traj::setMap(shared_ptr<SDFMap>& map)
  {
    this->mapping_ = map;
    this->traj_generator_->setMap(this->mapping_);

    return;
  }

  /**
   * @brief traj_id_ -> -1 for initial img traj while 0 for initial pc traj
   */
  void Init_Traj::run()
  {
    if (this->traj_id_ > 0) return;

    rclcpp::Time traj_start_time = node_->now() + rclcpp::Duration::from_seconds(3.0);
    // * Process Start Pose
    Eigen::VectorXd start_pose(5);

    Eigen::Vector3d plan_pos(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);

    tf2::Quaternion tfQuat_body;
    tf2::fromMsg(this->odom_.pose.pose.orientation, tfQuat_body);
    tf2::Matrix3x3 m_body(tfQuat_body);
    double roll_body, pitch_body, yaw_body;
    m_body.getRPY(roll_body, pitch_body, yaw_body);

    tf2::Quaternion tfQuat;
    tf2::fromMsg(this->gimbal_.quaternion, tfQuat);
    tf2::Matrix3x3 m(tfQuat);
    double roll_gimbal, pitch_gimbal, yaw_gimbal;
    m.getRPY(roll_gimbal, pitch_gimbal, yaw_gimbal);

    start_pose << plan_pos(0), plan_pos(1), plan_pos(2), pitch_gimbal, yaw_body;
    this->waypoints_.push_back(start_pose);
    this->indicators_.push_back(false);

    // * Read Initial Waypoints
    this->waypoints_.insert(this->waypoints_.end(), this->paths_[this->num_path_].begin(), this->paths_[this->num_path_].end());
    this->indicators_.insert(this->indicators_.end(), this->paths_indicators_[this->num_path_].begin(), this->paths_indicators_[this->num_path_].end());

    if ((int)waypoints_.size() == 1)
    {
      RCLCPP_WARN(rclcpp::get_logger("flyco.hierarchical_coverage_planner"), "\033[41;37m[FlyCo]\033[47;31m No Initial Waypoints Found! \033[0m");
      return;
    }

    // * Interpolate Path
    vector<Eigen::VectorXd> updated_waypoints;
    vector<bool> updated_indicators;
    for (int i=0; i<(int)this->waypoints_.size()-1; i++)
    {
      updated_waypoints.push_back(this->waypoints_[i]);
      updated_indicators.push_back(this->indicators_[i]);

      Eigen::VectorXd p1 = this->waypoints_[i];
      Eigen::VectorXd p2 = this->waypoints_[i+1];

      double dist = (p2.head(3) - p1.head(3)).norm();
      int num = ceil(dist / 1.0);
      double step = dist / num;

      vector<Eigen::VectorXd> piece_waypts;
      path_tools::pieceInterpolate(p1, p2, step, piece_waypts);
      if (piece_waypts.empty()) continue;

      vector<bool> piece_indicators(piece_waypts.size(), true);
      updated_waypoints.insert(updated_waypoints.end(), piece_waypts.begin(), piece_waypts.end());
      updated_indicators.insert(updated_indicators.end(), piece_indicators.begin(), piece_indicators.end());
    }
    updated_waypoints.push_back(this->waypoints_.back());
    updated_indicators.push_back(this->indicators_.back());

    this->waypoints_.clear();
    this->indicators_.clear();
    this->waypoints_ = updated_waypoints;
    this->indicators_ = updated_indicators;

    // * Generate Trajectory
    Eigen::Vector3d now_posi = this->waypoints_.front().head(3);
    vector<Eigen::Vector3d> wps;
    vector<bool> wps_waypt_indicators;
    vector<double> pitchs, yaws;

    wps.resize((int)this->waypoints_.size()-1);
    wps_waypt_indicators.resize((int)this->waypoints_.size()-1);
    pitchs.resize((int)this->waypoints_.size());
    yaws.resize((int)this->waypoints_.size());

    for (int i=0; i<(int)this->waypoints_.size(); ++i)
    {
      if (i > 0)
      {
        wps[i-1] = this->waypoints_[i].head(3);
        wps_waypt_indicators[i-1] = this->indicators_[i];
      }  
      pitchs[i] = this->waypoints_[i](3);
      yaws[i] = this->waypoints_[i](4);
    }

    voxel_map::VoxelMap vmap;
    double sfc = 0.0;
    this->traj_generator_->visFlag = true;
    this->traj_generator_->HCTraj(now_posi, wps, wps_waypt_indicators, pitchs, yaws, vmap, sfc, 1.7, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    Trajectory<7> pos_traj = this->traj_generator_->minco_traj;
    Trajectory<7> ori_traj = this->traj_generator_->minco_orientation_traj;

    // * Publish Initial Trajectory
    quadrotor_msgs::msg::PolynomialTrajGroup traj_msg;
    this->trajConverter(pos_traj, ori_traj, traj_msg, traj_start_time, this->traj_id_);
    this->traj_pub_->publish(traj_msg);

    if (this->traj_id_ >= 0)
    {
      // * Release
      this->traj_generator_.reset(new TrajGenerator);
      // DDS delivery is asynchronous: retain the writer after this final sample.
      this->joy_sub_.reset();
      this->odom_sub_.reset();
      this->gimbal_sub_.reset();
    }

    RCLCPP_INFO(rclcpp::get_logger("flyco.task_controls"), "\033[41;37m[FlyCo]\033[47;31m Initial -> [ %d ]-th Trajectory Generation...... \033[0m", this->traj_id_);
    rclcpp::Duration sleep_time = rclcpp::Duration::from_seconds(0.5);
    rclcpp::sleep_for(std::chrono::nanoseconds(sleep_time.nanoseconds()));
    this->finish_init_ = false;
    this->traj_id_++;
    this->num_path_++;
    this->waypoints_.clear();
    this->indicators_.clear();
    if (this->traj_generator_) this->traj_generator_->reset();

    return;
  }

  void Init_Traj::readInitialPoints()
  {
    string package_path = (ament_index_cpp::get_package_share_directory("flyco_ros2") + "/vendor/planner_simulator/src/flyco_planner_manager");
    string filename;

    if (this->en_inherit_)
    {
      filename = package_path + "/inherit_data/inherit_point.yaml";
      YAML::Node config = YAML::LoadFile(filename);
      vector<Eigen::VectorXd> waypts;
      vector<bool> indicators;
      for (const auto& point : config["waypoints"])
      {
        Eigen::VectorXd wp(5);
        
        wp << point[0].as<double>(), point[1].as<double>(), point[2].as<double>(), point[3].as<double>()*M_PI/180.0, point[4].as<double>()*M_PI/180.0;

        waypts.push_back(wp);
        indicators.push_back(false);
      }
      this->paths_.push_back(waypts);
      this->paths_indicators_.push_back(indicators);

      this->path_size_ = (int)this->paths_.size();
    }
    else
    {
      filename = this->init_path_file_;
      YAML::Node config = YAML::LoadFile(filename);
      for (const auto& path : config["path"])
      {
        vector<Eigen::VectorXd> waypts;
        vector<bool> indicators;
        for (const auto& point : path["waypoints"])
        {
          Eigen::VectorXd wp(5);
          
          wp << point[0].as<double>(), point[1].as<double>(), point[2].as<double>(), point[3].as<double>()*M_PI/180.0, point[4].as<double>()*M_PI/180.0;

          waypts.push_back(wp);
          indicators.push_back(false);
        }
        this->paths_.push_back(waypts);
        this->paths_indicators_.push_back(indicators);
      }

      this->path_size_ = (int)this->paths_.size();
    }

    return;
  }

  void Init_Traj::trajConverter(const Trajectory<7> &pos, const Trajectory<7> &ori, quadrotor_msgs::msg::PolynomialTrajGroup &msg, const rclcpp::Time &cur_stamp, int &traj_id)
  {
    msg.trajectory_id = traj_id;
    msg.header.stamp = cur_stamp;
    msg.action = msg.ACTION_ADD;
    msg.start_time = cur_stamp;

    int pn_pos = pos.getPieceNum();
    for (int i=0; i<pn_pos; ++i)
    {
      quadrotor_msgs::msg::PolynomialMatrix piece_pos;
      piece_pos.num_dim = pos[i].getDim();
      piece_pos.num_order = pos[i].getDegree();
      piece_pos.duration = pos[i].getDuration();
      auto cMat = pos[i].getCoeffMat();
      piece_pos.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
      msg.pos_traj.push_back(piece_pos);
    }

    int pn_ori = ori.getPieceNum();
    for (int i=0; i<pn_ori; ++i)
    {
      quadrotor_msgs::msg::PolynomialMatrix piece_ori;
      piece_ori.num_dim = ori[i].getDim();
      piece_ori.num_order = ori[i].getDegree();
      piece_ori.duration = ori[i].getDuration();
      auto cMat = ori[i].getCoeffMat();
      piece_ori.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
      msg.ori_traj.push_back(piece_ori);
    }

    return;
  }

  void Init_Traj::runCallback()
  {
    if (this->start_gen_ && !this->finish_init_)
    {
      tf2::Quaternion body, gimbal;
      tf2::fromMsg(this->odom_.pose.pose.orientation, body);
      tf2::fromMsg(this->gimbal_.quaternion, gimbal);
      const double body_norm = body.length2(), gimbal_norm = gimbal.length2();
      if (!std::isfinite(body_norm) || !std::isfinite(gimbal_norm) ||
          body_norm < 0.25 || body_norm > 2.25 || gimbal_norm < 0.25 || gimbal_norm > 2.25)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("flyco.hierarchical_coverage_planner"), flyco::steady_logging_clock(), static_cast<int64_t>((5.0) * 1000), "Waiting for valid odometry and gimbal poses before initial trajectory");
        return;
      }
      this->start_gen_ = false;
      this->finish_init_ = true;
      this->run();
    }

    return;
  }

  void Init_Traj::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy)
  {
    sensor_msgs::msg::Joy joy_msg;
    if (Joy) joy_msg = *Joy;

    // Button 'Y' to start initial trajectory generation
    if (joy_msg.buttons[3] == true) 
    {
      this->start_gen_ = true;
      RCLCPP_INFO(rclcpp::get_logger("flyco.task_controls"), "[2 / Y] Initial flight requested.");
    }

    return;
  }

  void Init_Traj::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
  {
    this->odom_ = *msg;

    return;
  }

  void Init_Traj::gimbalCallback(const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    this->gimbal_ = *msg;

    return;
  }

}
