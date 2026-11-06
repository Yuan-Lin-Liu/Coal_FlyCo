#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <flyco_ros2/parameters.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
 * Description  :    This file is the execution file of mapping and planning part in FlyCo.
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

#include "flyco_planner_manager/flyco_planner_manager.h"
#include "flyco_planner_manager/logger.hpp"
#include "backward.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <boost/circular_buffer.hpp>
namespace backward
{
  backward::SignalHandling sh;
}

using namespace std;
using std::unique_ptr;
using namespace flyco;

class FlyCoPlannerApplication {
  rclcpp::Node::SharedPtr node_;
public:
  explicit FlyCoPlannerApplication(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}
private:
/* Odom */
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, geometry_msgs::msg::QuaternionStamped> SyncPolicyPoseGimbal;
typedef unique_ptr<message_filters::Synchronizer<SyncPolicyPoseGimbal>> SynchronizerPoseGimbal;

unique_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_ = nullptr;
unique_ptr<message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>> gimbal_sub_ = nullptr;
SynchronizerPoseGimbal sync_odom_gimbal_;

/* Buffer */
boost::circular_buffer<nav_msgs::msg::Odometry::ConstSharedPtr> buffer_odom_;
boost::circular_buffer<geometry_msgs::msg::QuaternionStamped::ConstSharedPtr> buffer_quat_;

/* Param */
bool body_pose_valid_ = false;
bool receive_odom_ = true, start_task_ = false, sim_flag_ = true;
double ext_x_ = 0.0, ext_y_ = 0.0, ext_z_ = 0.0;
Eigen::Vector3d cam2body_t_vector_;
double odom_fps_ = 1.0, signal_task_fps_ = 1.0;
Eigen::VectorXd start_pose_ = Eigen::VectorXd(5);

/* Indicator */
int indi_num_ = -1;
vector<Eigen::VectorXd> indicators_;

/* ROS Service */
unique_ptr<rclcpp::WallRate> signal_task_rate_ = nullptr;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr indicator_pub_;

/* Joy Trigger */
bool last_lb_pressed_ = false;
const int JOY_LB_BUTTON = 4;

/* Utils */
void indicatorCallback()
{
  if (start_task_ == true) return;

  double scale = 1.0;

  visualization_msgs::msg::MarkerArray mk_array;

  for (int i=0; i<indi_num_; ++i)
  {
    Eigen::VectorXd indi_pt = indicators_[i];

    Eigen::Matrix3d Rwb_y, Rwb_p;
    Rwb_y << cos(indi_pt(4)), -sin(indi_pt(4)), 0,
            sin(indi_pt(4)), cos(indi_pt(4)), 0,
            0, 0, 1;
    Rwb_p << cos(indi_pt(3)), 0.0, -sin(indi_pt(3)), 
            0, 1, 0,
            sin(indi_pt(3)), 0.0, cos(indi_pt(3));
    
    Eigen::Vector3d position(indi_pt(0), indi_pt(1), indi_pt(2));
    Eigen::Vector3d orientation = position + Rwb_y*Rwb_p*Eigen::Vector3d(scale, 0.0, 0.0);
    
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = node_->now();
    mk.id = i;
    mk.ns = "selection_indicator";
    mk.type = visualization_msgs::msg::Marker::ARROW;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.4;
    mk.scale.z = 0.3;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;

    geometry_msgs::msg::Point pt_;
    pt_.x = position(0);
    pt_.y = position(1);
    pt_.z = position(2);
    mk.points.push_back(pt_);

    pt_.x = orientation(0);
    pt_.y = orientation(1);
    pt_.z = orientation(2);
    mk.points.push_back(pt_);

    mk_array.markers.push_back(mk);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "world";
    text_marker.header.stamp = node_->now();
    text_marker.id = i;
    text_marker.ns = "selection_indicator_text";
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.text = to_string(i);
    text_marker.pose.position.x = position(0);
    text_marker.pose.position.y = position(1);
    text_marker.pose.position.z = position(2);
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.x = scale;
    text_marker.scale.y = scale;
    text_marker.scale.z = scale;
    text_marker.color.r = 0.0;
    text_marker.color.g = 0.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;

    mk_array.markers.push_back(text_marker);
  }
  
  indicator_pub_->publish(mk_array);

  return;
}

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose, const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& quat)
{
  if (receive_odom_ == true)
  {
    buffer_odom_.push_back(pose);
    buffer_quat_.push_back(quat);
  }

  return;
}

void updatePoseCallback()
{
  if (!receive_odom_) return;
  if (buffer_odom_.empty() || buffer_quat_.empty()) return;

  nav_msgs::msg::Odometry::ConstSharedPtr pose = buffer_odom_.back();
  geometry_msgs::msg::QuaternionStamped::ConstSharedPtr quat = buffer_quat_.back();

  // * Process Odometry
  tf2::Quaternion tfQuat;
  tf2::fromMsg(quat->quaternion, tfQuat);
  tf2::Matrix3x3 m(tfQuat);
  double roll_gimbal, pitch_gimbal, yaw_gimbal;
  m.getRPY(roll_gimbal, pitch_gimbal, yaw_gimbal);

  Eigen::Vector3d body2world_t_vector;
  Eigen::Matrix3d body2world_R_matrix;
  Eigen::Quaterniond body2world_R_quat;

  body2world_t_vector(0) = pose->pose.pose.position.x; // in world coordinate
  body2world_t_vector(1) = pose->pose.pose.position.y; // in world coordinate
  body2world_t_vector(2) = pose->pose.pose.position.z; // in world coordinate

  tf2::Quaternion tfQuat_body;
  tf2::fromMsg(pose->pose.pose.orientation, tfQuat_body);
  tf2::Matrix3x3 m_body(tfQuat_body);
  double roll_body, pitch_body, yaw_body;
  m_body.getRPY(roll_body, pitch_body, yaw_body);

  body2world_R_matrix = Eigen::AngleAxisd(yaw_body, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(pitch_body, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(roll_body, Eigen::Vector3d::UnitX());
  body2world_R_quat = Eigen::Quaterniond(body2world_R_matrix);

  if (sim_flag_ == true)
  {
  // ! AirSim Simulation
  yaw_gimbal += yaw_body;
  pitch_gimbal += pitch_body;
  roll_gimbal = 0.0; // ? Fixed with Drone
  // ! ---------------------
  }

  Eigen::Vector3d camera_pos = body2world_t_vector;
  double pitch = pitch_gimbal;
  double yaw = yaw_body;

  // * Current Pose
  start_pose_ << camera_pos(0), camera_pos(1), camera_pos(2), pitch, yaw;
  body_pose_valid_ = true;

  return;
}

void selectStartFromJoystick()
{
  if (start_task_ == false)
  {
    if (!body_pose_valid_) { RCLCPP_WARN(node_->get_logger(), "Wait for a synchronized body/gimbal pose before Initialize"); return; }
    start_task_ = true;
    RCLCPP_INFO(rclcpp::get_logger("flyco.task_controls"), "\033[41;37m[FlyCo]\033[47;31m Successfully select the initialization from \n (%lf, %lf, %lf) in [World ENU] frame with pitch %lf deg and yaw %lf deg! \033[0m", start_pose_(0), start_pose_(1), start_pose_(2), start_pose_(3)*180.0/M_PI, start_pose_(4)*180.0/M_PI);
  }

  return;
}

void joyTriggerCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& Joy)
{
  if (!Joy) return;

  bool lb_pressed = Joy->buttons.size() > JOY_LB_BUTTON && Joy->buttons[JOY_LB_BUTTON] == true;

  if (lb_pressed && !last_lb_pressed_)
  {
    RCLCPP_INFO(rclcpp::get_logger("flyco.task_controls"), "\033[43;37m[FlyCo Joy]\033[47;33m LB pressed: select current pose as start. \033[0m");
    selectStartFromJoystick();
  }

  last_lb_pressed_ = lb_pressed;

  return;
}


public:
  int run() {

  
  const auto& nh = node_;
  
  // * Initialization
  sim_flag_ = flyco::parameter_or(nh, "flyco_planner.sim", true);
  odom_fps_ = flyco::parameter_or(nh, "flyco_planner.odom_fps", 1.0);
  signal_task_fps_ = flyco::parameter_or(nh, "flyco_planner.st_fps", 1.0);
  ext_x_ = flyco::parameter_or(nh, "flyco_planner.extrinsic_x", 0.0);
  ext_y_ = flyco::parameter_or(nh, "flyco_planner.extrinsic_y", 0.0);
  ext_z_ = flyco::parameter_or(nh, "flyco_planner.extrinsic_z", 0.0);
  cam2body_t_vector_ << ext_x_, ext_y_, ext_z_;
  double odom_time = 1.0/odom_fps_;

  indi_num_ = flyco::parameter_or(nh, "flyco_planner.indi_num", -1);
  for (int i=0; i<3; ++i)
  {
    Eigen::VectorXd indi_pt(5);
    indi_pt(0) = flyco::parameter_or(nh, "flyco_planner.indi_x_"+to_string(i), 0.0);
    indi_pt(1) = flyco::parameter_or(nh, "flyco_planner.indi_y_"+to_string(i), 0.0);
    indi_pt(2) = flyco::parameter_or(nh, "flyco_planner.indi_z_"+to_string(i), 0.0);
    indi_pt(3) = flyco::parameter_or(nh, "flyco_planner.indi_pitch_"+to_string(i), 0.0);
    indi_pt(4) = flyco::parameter_or(nh, "flyco_planner.indi_yaw_"+to_string(i), 0.0);
    indicators_.push_back(indi_pt);
  }

  int num = 20;
  buffer_odom_ = boost::circular_buffer<nav_msgs::msg::Odometry::ConstSharedPtr>(num);
  buffer_quat_ = boost::circular_buffer<geometry_msgs::msg::QuaternionStamped::ConstSharedPtr>(num);

  signal_task_rate_ = make_unique<rclcpp::WallRate>(signal_task_fps_);
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::msg::Odometry>(nh, "/flyco_planner/body_pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  gimbal_sub_.reset(new message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped>(nh, "/flyco_planner/gimbal_pose", rclcpp::QoS(10).reliable().get_rmw_qos_profile()));
  sync_odom_gimbal_.reset(new message_filters::Synchronizer<SyncPolicyPoseGimbal>(SyncPolicyPoseGimbal(20), *odom_sub_, *gimbal_sub_));
  sync_odom_gimbal_->registerCallback(std::bind(&FlyCoPlannerApplication::odomCallback, this, std::placeholders::_1, std::placeholders::_2));
  indicator_pub_ = nh->create_publisher<visualization_msgs::msg::MarkerArray>("/flyco_planner/indicator", rclcpp::QoS(1).reliable());
  rclcpp::TimerBase::SharedPtr update_pose_timer = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(odom_time), std::bind(&FlyCoPlannerApplication::updatePoseCallback, this));
  rclcpp::TimerBase::SharedPtr indicator_timer = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(odom_time), std::bind(&FlyCoPlannerApplication::indicatorCallback, this));

  flyco::logger::deviceInfo();

  // * Waiting User's Start Initialization
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_trigger_sub = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::QoS(50).reliable(), std::bind(&FlyCoPlannerApplication::joyTriggerCallback, this, std::placeholders::_1));
  RCLCPP_INFO(rclcpp::get_logger("flyco.task_controls"), "[Keys] 1/LB: initialize; 2/Y: initial flight; send target prompt; 3/X: enable planner; 4/RB: scan.");
  while (rclcpp::ok() && start_task_ == false)
  {
    rclcpp::spin_some(node_);
    signal_task_rate_->sleep();
  }

  // * Task Starting
  update_pose_timer->cancel();
  indicator_timer->cancel();
  sync_odom_gimbal_.reset();
  odom_sub_.reset();
  gimbal_sub_.reset();

  FlyCoPlanManager flyco_planner_manager;
  flyco_planner_manager.setStart(start_pose_);
  flyco_planner_manager.init(nh);
  RCLCPP_INFO(rclcpp::get_logger("flyco.flyco_planner_manager"), "\033[41;37m[FlyCo]\033[47;31m WAITING your trigger...... \033[0m");
  flyco_planner_manager.run();

  rclcpp::sleep_for(std::chrono::nanoseconds(rclcpp::Duration::from_seconds(1.0).nanoseconds()));
  rclcpp::spin(node_);

  return 0;

  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("flyco_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  int result;
  {
    FlyCoPlannerApplication application(node);
    result = application.run();
  }
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
