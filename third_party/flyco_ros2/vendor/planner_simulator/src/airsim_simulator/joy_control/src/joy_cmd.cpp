#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <flyco_ros2/parameters.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
 * Description  :    This file is the manual controller using joystick.
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

#include "airsim_ros_pkgs/msg/gimbal_angle_euler_cmd.hpp"
#include "active_perception/perception_utils.h"
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/joy.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using std::unique_ptr;
using flyco::PerceptionUtils;

class JoystickCommandApplication {
  rclcpp::Node::SharedPtr node_;
public:
  explicit JoystickCommandApplication(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}
private:
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, geometry_msgs::msg::QuaternionStamped> SyncPolicyPoseGimbal;
typedef unique_ptr<message_filters::Synchronizer<SyncPolicyPoseGimbal>> SynchronizerPoseGimbal;

unique_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
unique_ptr<message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>> gimbal_sub_;
SynchronizerPoseGimbal sync_odom_gimbal_;

/* ROS Service */
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
  rclcpp::Publisher<airsim_ros_pkgs::msg::GimbalAngleEulerCmd>::SharedPtr gimbal_cmd_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exec_traj_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_pub;
quadrotor_msgs::msg::PositionCommand cmd;
airsim_ros_pkgs::msg::GimbalAngleEulerCmd gimbal_cmd;
sensor_msgs::msg::Joy joy_;
Eigen::Quaterniond current_orientation;
unique_ptr<rclcpp::WallRate> rate = nullptr;
unique_ptr<PerceptionUtils> percep_utils_ = nullptr;

/* Param */
double ctrl_time_;
double transition_;
double kp_x_, kp_y_, kp_z_, kp_yaw_, kp_pitch_;
double v_max_, a_max_, ydot_max_;
double pitch_upper_, pitch_lower_;
double init_z_;
bool start_joy_ = false;

/* Data */
double current_body_x = 0.0, current_body_y = 0.0, current_body_z = 0.0;
double current_body_vel_x = 0.0, current_body_vel_y = 0.0, current_body_vel_z = 0.0;
double currennt_body_yaw = 0.0;
double current_gimbal_pitch = 0.0;

template <typename T>
T flycoClamp(T v, T lo, T hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

vector<Eigen::Vector3d> exec_traj_;

/* Func */
double smoothStep(double edge0, double edge1, double x) 
{
    // Scale, bias and saturate x to 0..1 range
    x = flycoClamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    // Evaluate polynomial
    return x * x * (3 - 2 * x);
}

double mapAxis(double value, double kp, double a_max, double transition_point)
{
    double abs_value = std::abs(value);
    if (abs_value < transition_point) 
    {
        return kp * a_max * std::pow(value, 3);
    } 
    else if (abs_value < transition_point + 0.1) 
    {
        double t = smoothStep(transition_point, transition_point + 0.1, abs_value);
        double cubic = std::pow(transition_point, 3);
        double linear = transition_point;
        double interpolated = (1 - t) * cubic + t * linear;
        return kp * a_max * (value >= 0 ? interpolated : -interpolated);
    } 
    else 
    {
        return kp * a_max * value;
    }
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose, const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& quat)
{
    current_body_x = pose->pose.pose.position.x;
    current_body_y = pose->pose.pose.position.y;
    current_body_z = pose->pose.pose.position.z;

    current_body_vel_x = pose->twist.twist.linear.x;
    current_body_vel_y = pose->twist.twist.linear.y;
    current_body_vel_z = pose->twist.twist.linear.z;
    
    current_orientation.w() = pose->pose.pose.orientation.w;
    current_orientation.x() = pose->pose.pose.orientation.x;
    current_orientation.y() = pose->pose.pose.orientation.y;
    current_orientation.z() = pose->pose.pose.orientation.z;

    tf2::Quaternion tfQuat_body;
    tf2::fromMsg(pose->pose.pose.orientation, tfQuat_body);
    tf2::Matrix3x3 m_body(tfQuat_body);
    double roll_body, pitch_body, yaw_body;
    m_body.getRPY(roll_body, pitch_body, yaw_body);

    currennt_body_yaw = yaw_body;

    tf2::Quaternion tfQuat;
    tf2::fromMsg(quat->quaternion, tfQuat);
    tf2::Matrix3x3 m(tfQuat);
    double roll_gimbal, pitch_gimbal, yaw_gimbal;
    m.getRPY(roll_gimbal, pitch_gimbal, yaw_gimbal);

    current_gimbal_pitch = pitch_gimbal;

    if (exec_traj_.empty())
    {
        Eigen::Vector3d cur_pos(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z);
        exec_traj_.push_back(cur_pos);
    }
    else
    {
        Eigen::Vector3d last_pos = exec_traj_.back();
        Eigen::Vector3d cur_pos(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z);
        double dist = (cur_pos - last_pos).norm();
        if (dist > 0.1)
            exec_traj_.push_back(cur_pos);
    }

    return;
}

void execVisCallback()
{   
    Eigen::Vector3d cur_pos(current_body_x, current_body_y, current_body_z);
    
    // * robot
    visualization_msgs::msg::Marker robot_vis;
    robot_vis.header.frame_id = "world";
    robot_vis.header.stamp = node_->now();
    robot_vis.id = 0;
    robot_vis.ns = "joy_robot_vis";
    robot_vis.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    robot_vis.color.r = 1.0;
    robot_vis.color.g = 0.0;
    robot_vis.color.b = 1.0;
    robot_vis.color.a = 1.0;
    robot_vis.scale.x = 1.0;
    robot_vis.scale.y = 1.0;
    robot_vis.scale.z = 1.0;
    robot_vis.mesh_resource = "package://vis_utils/DJI_M30T_S.dae";

    robot_vis.action = visualization_msgs::msg::Marker::DELETE;
    robot_pub->publish(robot_vis);

    double yaw_drone = currennt_body_yaw - M_PI/2.0;

    robot_vis.pose.orientation.x = 0.0;
    robot_vis.pose.orientation.y = 0.0;
    robot_vis.pose.orientation.z = sin(0.5*yaw_drone);
    robot_vis.pose.orientation.w = cos(0.5*yaw_drone);
    robot_vis.pose.position.x = cur_pos(0);
    robot_vis.pose.position.y = cur_pos(1);
    robot_vis.pose.position.z = cur_pos(2);
    robot_vis.action = visualization_msgs::msg::Marker::ADD;

    robot_pub->publish(robot_vis);
    
    // * FoV
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = 0;
    mk.ns = "joy_fov_vis";
    mk.type = visualization_msgs::msg::Marker::LINE_LIST;
    mk.color.r = 0.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.08;
    mk.scale.y = 0.08;
    mk.scale.z = 0.08;
    mk.pose.orientation.w = 1.0;

    mk.action = visualization_msgs::msg::Marker::DELETE;
    fov_pub->publish(mk);

    percep_utils_->setPose_PY(cur_pos, current_gimbal_pitch, currennt_body_yaw);
    vector<Eigen::Vector3d> list1, list2;
    percep_utils_->getFOV_PY(list1, list2);

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
    fov_pub->publish(mk);

    // * exec trajectory
    if (exec_traj_.empty())
    return;
    
    nav_msgs::msg::Path traj_msg_;
    traj_msg_.header.stamp = node_->now();
    traj_msg_.header.frame_id = "world";

    for (const auto& point : exec_traj_)
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

    exec_traj_pub->publish(traj_msg_);

    return;
}

void timeCallback()
{   
    rclcpp::Time cur_time = node_->now();
    
    if (!start_joy_)
        return;

    cmd.kx = { 7.0, 7.0, 8.0 };
    cmd.kv = { 4.4, 4.4, 5.0 };
    cmd.header.stamp = cur_time;
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = 0;

    cmd.jerk.x = 0.0;
    cmd.jerk.y = 0.0;
    cmd.jerk.z = 0.0;

    double delta_x = 0, delta_y = 0, delta_z = 0, delta_yaw = 0;
    double pitch_up = 0, pitch_down = 0, pitch_cmd = 0;
    double position_scaling_x = v_max_ * ctrl_time_, position_scaling_y = v_max_ * ctrl_time_, position_scaling_z = v_max_ * ctrl_time_;

    if (!joy_.axes.empty())
    {
        delta_x = mapAxis(joy_.axes[1], kp_x_, position_scaling_x, transition_);
        delta_y = mapAxis(joy_.axes[0], kp_y_, position_scaling_y, transition_);
        delta_z = mapAxis(joy_.axes[4], kp_z_, position_scaling_z, transition_);
        delta_yaw = kp_yaw_ * ydot_max_ * std::pow(joy_.axes[3], 2) * (joy_.axes[3] >= 0 ? 1 : -1);

        pitch_up = -kp_pitch_ * (joy_.axes[2] - 1.0);
        pitch_down = -kp_pitch_ * (joy_.axes[5] - 1.0);
        pitch_cmd = pitch_up - pitch_down;
    }

    // * Local to World Frame
    Eigen::Vector3d delta_body(delta_x, delta_y, delta_z);
    Eigen::Vector3d delta_world = current_orientation * delta_body;
    delta_x = delta_world.x();
    delta_y = delta_world.y();
    delta_z = delta_world.z();

    double new_x = cmd.position.x + delta_x;
    double new_y = cmd.position.y + delta_y;
    double new_z = cmd.position.z + delta_z;

    Eigen::Vector3d des_vel(delta_x / ctrl_time_, delta_y / ctrl_time_, delta_z / ctrl_time_);
    double des_vel_norm = des_vel.norm();
    des_vel = des_vel_norm > v_max_ ? des_vel / des_vel_norm * v_max_ : des_vel;

    cmd.velocity.x = des_vel(0);
    cmd.velocity.y = des_vel(1);
    cmd.velocity.z = des_vel(2);

    cmd.position.x = new_x;
    cmd.position.y = new_y;
    cmd.position.z = new_z;

    cmd.yaw += delta_yaw * ctrl_time_;

    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    
    cmd.yaw_dot = delta_yaw;

    gimbal_cmd.header.stamp = cur_time;
    gimbal_cmd.roll = 0.0;
    gimbal_cmd.pitch += pitch_cmd;
    gimbal_cmd.pitch = std::max(pitch_lower_, std::min(pitch_upper_, gimbal_cmd.pitch));
    gimbal_cmd.yaw = 0.0;

    gimbal_cmd_pub->publish(gimbal_cmd);
    pos_cmd_pub->publish(cmd);

    rate->sleep();
}

void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy)
{
    if (Joy)
      joy_ = *Joy;
    
    if (joy_.buttons[0] == true)
    {
        start_joy_ = true;
        RCLCPP_WARN(rclcpp::get_logger("flyco.joy_control"), "[Joy Node]: Start Joy Control......");

        // Initialize command
        cmd.position.x = current_body_x;
        cmd.position.y = current_body_y;
        cmd.position.z = current_body_z;
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        cmd.jerk.x = 0.0;
        cmd.jerk.y = 0.0;
        cmd.jerk.z = 0.0;
        cmd.yaw = currennt_body_yaw;
        cmd.yaw_dot = 0.0;

        gimbal_cmd.roll = 0.0;
        gimbal_cmd.pitch = current_gimbal_pitch*180.0/M_PI;
        gimbal_cmd.yaw = 0.0;
    }
    else if (joy_.buttons[1] == true)
    {
        start_joy_ = false;
        RCLCPP_WARN(rclcpp::get_logger("flyco.joy_control"), "[Joy Node]: Stop Joy Control......");
    }

    return;
}


public:
  int run() {

    
    const auto& n = node_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
  rclcpp::SubscriptionBase::SharedPtr odom_sub;

    RCLCPP_INFO(rclcpp::get_logger("flyco.joy_control"), "[Joy Node]: Please trigger the 'LT' and 'RT' keys at the same time to unlock the device......");
    RCLCPP_INFO(rclcpp::get_logger("flyco.joy_control"), "[Joy Node]: Press 'A' button to start the joy control......");
    RCLCPP_INFO(rclcpp::get_logger("flyco.joy_control"), "[Joy Node]: Press 'B' button to stop the joy control......");

    ctrl_time_ = flyco::parameter_or(n, "time", -1.0);
    transition_ = flyco::parameter_or(n, "transition", 0.5);
    kp_x_ = flyco::parameter_or(n, "kp_x", -1.0);
    kp_y_ = flyco::parameter_or(n, "kp_y", -1.0);
    kp_z_ = flyco::parameter_or(n, "kp_z", -1.0);
    kp_yaw_ = flyco::parameter_or(n, "kp_yaw", -1.0);
    kp_pitch_ = flyco::parameter_or(n, "kp_pitch", -1.0);
    v_max_ = flyco::parameter_or(n, "max_vel", -1.0);
    a_max_ = flyco::parameter_or(n, "max_acc", -1.0);
    ydot_max_ = flyco::parameter_or(n, "max_yd", -1.0);
    pitch_upper_ = flyco::parameter_or(n, "pitch_upper", -1.0);
    pitch_lower_ = flyco::parameter_or(n, "pitch_lower", -1.0);
    init_z_ = flyco::parameter_or(n, "init_z", -1.0);

    double top_angle, left_angle, right_angle, max_dist, vis_dist;
    top_angle = flyco::parameter_or(n, "top_angle", -1.0);
    left_angle = flyco::parameter_or(n, "left_angle", -1.0);
    right_angle = flyco::parameter_or(n, "right_angle", -1.0);
    max_dist = flyco::parameter_or(n, "max_dist", -1.0);
    vis_dist = flyco::parameter_or(n, "vis_dist", -1.0);
    const auto& nh = node_;
    flyco::set_parameter(nh, "perception_utils.top_angle", top_angle);
    flyco::set_parameter(nh, "perception_utils.left_angle", left_angle);
    flyco::set_parameter(nh, "perception_utils.right_angle", right_angle);
    flyco::set_parameter(nh, "perception_utils.max_dist", max_dist);
    flyco::set_parameter(nh, "perception_utils.vis_dist", vis_dist);
    percep_utils_.reset(new PerceptionUtils);
    percep_utils_->init(nh);

    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(n, "/airsim_node/drone_1/odom_local_enu", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
    gimbal_sub_.reset(new message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>(n, "/gimbal_pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
    sync_odom_gimbal_.reset(new message_filters::Synchronizer<SyncPolicyPoseGimbal>(SyncPolicyPoseGimbal(20), *odom_sub_, *gimbal_sub_));
    sync_odom_gimbal_->registerCallback(std::bind(&JoystickCommandApplication::odomCallback, this, std::placeholders::_1, std::placeholders::_2));

    // * Initialize cmds
    cmd.kx = { 7.0, 7.0, 8.0 };
    cmd.kv = { 4.4, 4.4, 5.0 };
    cmd.header.stamp = node_->now();
    cmd.header.frame_id = "world";
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = 0;

    gimbal_cmd.header.stamp = node_->now();
    gimbal_cmd.camera_name = "front_center";
    gimbal_cmd.vehicle_name = "drone_1";

    double fps = 1.0 / ctrl_time_;
    rate = make_unique<rclcpp::WallRate>(fps);

    pos_cmd_pub = n->create_publisher<quadrotor_msgs::msg::PositionCommand>("/joy_ctrl/pos_cmd", rclcpp::QoS(50).reliable());
    gimbal_cmd_pub = n->create_publisher<airsim_ros_pkgs::msg::GimbalAngleEulerCmd>("/joy_ctrl/gimbal_cmd", rclcpp::QoS(50).reliable());
    exec_traj_pub = n->create_publisher<nav_msgs::msg::Path>("/joy_ctrl/exec_traj", rclcpp::QoS(1).reliable().transient_local());
    robot_pub = n->create_publisher<visualization_msgs::msg::Marker>("/joy_ctrl/robot", rclcpp::QoS(1).reliable());
    fov_pub = n->create_publisher<visualization_msgs::msg::Marker>("/joy_ctrl/fov", rclcpp::QoS(1).reliable());

    rclcpp::TimerBase::SharedPtr exec_timer_ = rclcpp::create_timer(n, n->get_clock(), rclcpp::Duration::from_seconds(ctrl_time_), std::bind(&JoystickCommandApplication::timeCallback, this));
    rclcpp::TimerBase::SharedPtr exec_vis_timer_ = rclcpp::create_timer(n, n->get_clock(), rclcpp::Duration::from_seconds(0.1), std::bind(&JoystickCommandApplication::execVisCallback, this));
    sub = n->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(50).reliable(), std::bind(&JoystickCommandApplication::joyCallback, this, std::placeholders::_1));
    
    rclcpp::sleep_for(std::chrono::nanoseconds(rclcpp::Duration::from_seconds(1.0).nanoseconds()));
    rclcpp::spin(node_);

    return 0;

  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("joy_cmd",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  int result;
  {
    JoystickCommandApplication application(node);
    result = application.run();
  }
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
