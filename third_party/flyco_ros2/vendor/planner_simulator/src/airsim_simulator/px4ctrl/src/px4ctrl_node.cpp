#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Sept. 2024
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the execution file of the PX4 control node for FlyCo
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

#include <rclcpp/rclcpp.hpp>
#include "PX4CtrlFSM.h"

//#include <quadrotor_msgs/msg/so3_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
//#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
//#include <std_msgs/msg/header.hpp>
//#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <signal.h>
#include "std_msgs/msg/float32.hpp"

PX4CtrlFSM* pFSM;

void mySigintHandler(int sig) {
    RCLCPP_INFO(rclcpp::get_logger("flyco.px4ctrl"), "[PX4Ctrl] exit...");
    rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("px4ctrl", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    const auto& nh = node;
    signal(SIGINT, mySigintHandler);
    Parameter_t param;
    Controller controller(param);
    controller.set_FCU_mode = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    HovThrKF hov_thr_kf(param);
    PX4CtrlFSM fsm(param, controller, hov_thr_kf, node);
    pFSM = &fsm;

    param.config_from_ros_handle(nh);
    param.init();//recompute the full thrust.
    fsm.hov_thr_kf.init();
    fsm.hov_thr_kf.set_hov_thr(param.hov_percent);//x(0) = hov_percent
    
    RCLCPP_INFO(rclcpp::get_logger("flyco.px4ctrl"), "Initial value for hov_thr set to %.2f/%.2f", fsm.hov_thr_kf.get_hov_thr(), param.mass * param.gra / param.full_thrust);
    RCLCPP_INFO(rclcpp::get_logger("flyco.px4ctrl"), "Hovering thrust kalman filter is %s.", param.hover.use_hov_percent_kf ? "used" : "NOT used");

    fsm.controller.config();//

    // rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub = nh->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::QoS(//                                                              10).reliable(), //                                                              std::bind(&State_Data_t::feed, &fsm.state_data, std::placeholders::_1));
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub =
        nh->create_subscription<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS(100).reliable(), std::bind(&Odom_Data_t::feed, &fsm.odom_data, std::placeholders::_1));


    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub = 
        nh->create_subscription<quadrotor_msgs::msg::PositionCommand>("~/cmd", rclcpp::QoS(100).reliable(), std::bind(&Command_Data_t::feed, &fsm.cmd_data, std::placeholders::_1));
    //All okay for both above

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub =
        nh->create_subscription<sensor_msgs::msg::Imu>("~/imu", rclcpp::QoS(100).reliable(), std::bind(&Imu_Data_t::feed, &fsm.imu_data, std::placeholders::_1));
    fsm.controller.ctrl_FCU_pub = nh->create_publisher<mavros_msgs::msg::AttitudeTarget>("/setpoint_raw/attitude", rclcpp::QoS(10).reliable());
    // fsm.controller.debug_roll_pub = nh->create_publisher<std_msgs::msg::Float32>("/debug_roll", rclcpp::QoS(10).reliable());
    // fsm.controller.debug_pitch_pub = nh->create_publisher<std_msgs::msg::Float32>("/debug_pitch", rclcpp::QoS(10).reliable());
    // fsm.traj_start_trigger_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/traj_start_trigger", rclcpp::QoS(10).reliable());
    rclcpp::WallRate r(param.ctrl_rate);
    // ---- process ----
    while (rclcpp::ok()) {
        r.sleep();
        rclcpp::spin_some(node);
        if(fsm.px4_init())
            fsm.process();
    }

    return 0;
}
