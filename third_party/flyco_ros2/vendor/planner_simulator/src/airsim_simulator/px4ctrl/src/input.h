#include <nav_msgs/msg/odometry.hpp>
/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Sept. 2024
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the header file of PX4 control input handling for FlyCo
 *                   simulation.
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

#ifndef __INPUT_H
#define __INPUT_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/msg/imu.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

class RC_Data_t {
  public:
    rclcpp::Clock::SharedPtr clock_;
    double mode;
    double gear;
    double last_mode;
    double last_gear;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};

    mavros_msgs::msg::RCIn msg;
    rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};

    bool is_command_mode;
    bool enter_command_mode;
    bool is_api_mode;
    bool enter_api_mode;    

    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;

    RC_Data_t();
    void check_validity();
    void feed(mavros_msgs::msg::RCIn::ConstSharedPtr pMsg);
};

class Odom_Data_t {
  public:
    rclcpp::Clock::SharedPtr clock_;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d w;
    Eigen::Quaterniond q;

    nav_msgs::msg::Odometry msg;
    rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
    bool odom_init;
    Odom_Data_t();
    void feed(nav_msgs::msg::Odometry::ConstSharedPtr pMsg);
};

class Imu_Data_t {
  public:
    rclcpp::Clock::SharedPtr clock_;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::msg::Imu msg;
    rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};
    bool imu_init;
    Imu_Data_t();
    void feed(sensor_msgs::msg::Imu::ConstSharedPtr pMsg);
};

class State_Data_t {
  public:
    rclcpp::Clock::SharedPtr clock_;
    mavros_msgs::msg::State current_state;

    State_Data_t();
    void feed(mavros_msgs::msg::State::ConstSharedPtr pMsg);
};

class Command_Data_t {
  public:
    rclcpp::Clock::SharedPtr clock_;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d jerk;
    double yaw;
    double head_rate;
    bool cmd_init;
    quadrotor_msgs::msg::PositionCommand msg;
    rclcpp::Time rcv_stamp{0, 0, RCL_ROS_TIME};

    Command_Data_t();
    void feed(quadrotor_msgs::msg::PositionCommand::ConstSharedPtr pMsg);
};

#endif
