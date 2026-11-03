#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Sept. 2024
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the implementation file of the PX4 control finite-state
 *                   machine for FlyCo simulation.
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

#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t& param_, Controller& controller_, HovThrKF& hov_thr_kf_, const rclcpp::Node::SharedPtr& node):
	node_(node), param(param_), controller(controller_), hov_thr_kf(hov_thr_kf_)
{
	state = MANUAL_CTRL;
  rc_data.clock_ = node_->get_clock();
  odom_data.clock_ = node_->get_clock();
  imu_data.clock_ = node_->get_clock();
  state_data.clock_ = node_->get_clock();
  cmd_data.clock_ = node_->get_clock();
	hover_pose.setZero();
}

void PX4CtrlFSM::process()
{
	rclcpp::Time now_time = node_->now();
	Controller_Output_t u;
	SO3_Controller_Output_t u_so3;

	// if (!rc_is_received(now_time))
	// {
	// 	RCLCPP_ERROR(rclcpp::get_logger("flyco.px4ctrl"), "RC lost for %3f seconds!!!!", (now_time - rc_data.rcv_stamp).seconds());
	// }

	// // cout << state << " " << rc_data.is_api_mode  << endl;

	// switch (state)
	// {
	// 	case MANUAL_CTRL:
	// 	{
	// 		if ( rc_data.enter_api_mode ) 
	// 		{
	// 			rc_data.enter_api_mode = false;
	// 			if ( odom_is_received(now_time) )
	// 			{
	// 				state = AUTO_HOVER;
	// 				controller.config_gain(param.hover_gain);
	// 				set_hov_with_odom();
	// 				toggle_offboard_mode(true);

	// 				RCLCPP_INFO(rclcpp::get_logger("flyco.px4ctrl"), "\033[32m[px4ctrl] Start AUTO_HOVER by px4ctrl!\033[32m");
	// 			}
	// 			else
	// 			{
	// 				RCLCPP_ERROR(rclcpp::get_logger("flyco.px4ctrl"), "[px4ctrl] Reject AUTO_HOVER. No odom!");
	// 			}
	// 		}

	// 		break;
	// 	}

	// 	case AUTO_HOVER:
	// 	{
	// 		if ( !rc_data.is_api_mode || !odom_is_received(now_time) )
	// 		{
	// 			state = MANUAL_CTRL;
	// 			toggle_offboard_mode(false);

	// 			RCLCPP_WARN(rclcpp::get_logger("flyco.px4ctrl"), "[px4ctrl] Return to manual control!");
	// 		}
	// 		else if ( rc_data.is_command_mode && cmd_is_received(now_time) )
	// 		{
	// 			if ( state_data.current_state.mode == "OFFBOARD" )
	// 			{
	// 				state = CMD_CTRL;
	// 				controller.config_gain(param.track_gain);
	// 				process_cmd_control(u, u_so3);
	// 				RCLCPP_INFO(rclcpp::get_logger("flyco.px4ctrl"), "\033[32m[px4ctrl] position commands received!\033[32m");
	// 			}
	// 		}
	// 		else
	// 		{
	// 			process_hover_control(u, u_so3);
	// 			if ( rc_data.enter_command_mode )
	// 			{
	// 				rc_data.enter_command_mode = false;
	// 				publish_trigger(odom_data.msg);
	// 			}
	// 		}

	// 		break;
	// 	}

	// 	case CMD_CTRL:
	// 	{
	// 		if ( !rc_data.is_api_mode || !odom_is_received(now_time) )
	// 		{
	// 			state = MANUAL_CTRL;
	// 			toggle_offboard_mode(false);
				
	// 			RCLCPP_WARN(rclcpp::get_logger("flyco.px4ctrl"), "[px4ctrl] Return to MANUAL_CTRL!");
	// 		}
	// 		else if ( !cmd_is_received(now_time) )
	// 		{
	// 			state = AUTO_HOVER;
    //         	controller.config_gain(param.hover_gain);
    //         	set_hov_with_odom();
	// 			process_hover_control(u, u_so3);
	// 			RCLCPP_WARN(rclcpp::get_logger("flyco.px4ctrl"), "[px4ctrl] Return to AUTO_HOVER!");
	// 		}
	// 		else
	// 		{
	// 			process_cmd_control(u, u_so3);
	// 		}
			
	// 		break;
	// 	}
	
	// 	default:
	// 		break;
	// }


	
	controller.config_gain(param.track_gain);
	process_cmd_control(u, u_so3);
	// u.thrust = std::max(u.thrust, 0.03);
	// align_with_imu(u);
	// u.yaw = cmd_data.yaw;
	controller.publish_ctrl(u, now_time);
	hov_thr_kf.simple_update(u.des_v_real, odom_data.v );
	// This line may not take effect according to param.hov.use_hov_percent_kf
	param.config_full_thrust(hov_thr_kf.get_hov_thr());
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::msg::Odometry& odom_msg)
{
	geometry_msgs::msg::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;
	
    traj_start_trigger_pub->publish(msg);
}

bool PX4CtrlFSM::rc_is_received(const rclcpp::Time& now_time)
{
	return (now_time - rc_data.rcv_stamp).seconds() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const rclcpp::Time& now_time)
{
	return (now_time - cmd_data.rcv_stamp).seconds() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const rclcpp::Time& now_time)
{
	return (now_time - odom_data.rcv_stamp).seconds() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const rclcpp::Time& now_time)
{
	return (now_time - imu_data.rcv_stamp).seconds() < param.msg_timeout.imu;
}

double PX4CtrlFSM::get_yaw_from_odom()
{
	return get_yaw_from_quaternion(odom_data.q);
}

void PX4CtrlFSM::process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.a = Vector3d::Zero();
	des.jerk = Vector3d::Zero();
	controller.update(des, odom_data, u, u_so3);

	//publish_desire(des);
}

void PX4CtrlFSM::process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.yaw = cmd_data.yaw;
	des.a = cmd_data.a;
	des.jerk = cmd_data.jerk;
	des.head_rate = cmd_data.head_rate;
	// RCLCPP_INFO_STREAM(rclcpp::get_logger("flyco.px4ctrl"), "desp: "<<des.p<<" desv: "<<des.v<<" desyaw: "<<des.yaw<<" desa "<<des.a);
	// RCLCPP_INFO_STREAM(rclcpp::get_logger("flyco.px4ctrl"), "odomp: "<<odom_data.p<<" odomv: "<<odom_data.v);
	// RCLCPP_INFO_STREAM(rclcpp::get_logger("flyco.px4ctrl"), "qo "<<odom_data.q.w()<<" q1 "<<odom_data.q.x() <<" q2 "<<odom_data.q.y()<<" q3 "<<odom_data.q.z());
	controller.update(des, odom_data, u, u_so3);
	// RCLCPP_INFO_STREAM(rclcpp::get_logger("flyco.px4ctrl"), "pitch: "<<u.pitch<<"roll: "<<u.roll<<" u.yaw: "<<u.yaw);

	//publish_desire(des);	
}

void PX4CtrlFSM::align_with_imu(Controller_Output_t& u)
{
	double imu_yaw = get_yaw_from_quaternion(imu_data.q); 
	double odom_yaw = get_yaw_from_odom();
	double des_yaw = u.yaw;
	// RCLCPP_INFO_STREAM(rclcpp::get_logger("flyco.px4ctrl"), "imu yaw: "<<imu_yaw<<" odom_yaw: "<<odom_yaw);
	u.yaw = yaw_add(yaw_add(des_yaw, -odom_yaw), imu_yaw); 

	//out << "imu_yaw=" << imu_yaw << " odom_yaw=" << odom_yaw << " des_yaw=" << des_yaw << " u.yaw=" << u.yaw << endl;
};

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_odom();
}

// void PX4CtrlFSM::publish_desire(const Desired_State_t& des)
// {
// 	geometry_msgs::msg::PoseStamped msg;
// 	msg.header = odom_data.msg.header;

// 	msg.pose.position.x = des.p(0);
// 	msg.pose.position.y = des.p(1);
// 	msg.pose.position.z = des.p(2);

// 	Eigen::Quaterniond q = yaw_to_quaternion(des.yaw);

// 	msg.pose.orientation.w = q.w();
// 	msg.pose.orientation.x = q.x();
// 	msg.pose.orientation.y = q.y();
// 	msg.pose.orientation.z = q.z();

// 	des_pose_pub.publish(msg);
// }

void PX4CtrlFSM::toggle_offboard_mode(bool enabled) {
  if (!controller.set_FCU_mode || !controller.set_FCU_mode->service_is_ready()) return;
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = enabled ? "OFFBOARD" : "ALTCTL";
  auto result = controller.set_FCU_mode->async_send_request(request);
  rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(5));
}
bool PX4CtrlFSM::px4_init(){
	return (odom_data.odom_init&&imu_data.imu_init&&cmd_data.cmd_init);
}
