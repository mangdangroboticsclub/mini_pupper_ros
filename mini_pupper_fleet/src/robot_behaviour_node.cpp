// SPDX-License-Identifier: Apache-2.0
//
// Copyright (c) 2025 MangDang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mini_pupper_fleet/robot_behaviour_node.hpp"

#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

RobotBehaviourNode::RobotBehaviourNode()
: Node("robot_behaviour_node")
{
	RCLCPP_INFO(this->get_logger(), "RobotBehaviourNode has started.");

	// timer
	last_tick_st_ = steady_clock_.now();
	control_timer_ = this->create_wall_timer(
		std::chrono::milliseconds(ControlPeriodMs),
		std::bind(&RobotBehaviourNode::control_loop_, this));

	auto qos_fleet_command = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
	fleet_command_subscription_ = this->create_subscription<mini_pupper_interfaces::msg::FleetCommand>(
		"/fleet_command", qos_fleet_command,
		std::bind(&RobotBehaviourNode::fleet_command_callback_, this, std::placeholders::_1));
	
    auto qos_ekf_pose = rclcpp::SensorDataQoS();
	ekf_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"ekf_pose", qos_ekf_pose,
		std::bind(&RobotBehaviourNode::ekf_pose_callback_, this, std::placeholders::_1));

	auto qos_cmd_vel = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
	cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_cmd_vel);
}

RobotBehaviourNode::Mode RobotBehaviourNode::decide_mode_(double vx_ref) const
{
	if (std::abs(vx_ref) < 1e-6) {
		return Mode::RotateOnly;
	}
	return Mode::MoveAndAlign;
}

void RobotBehaviourNode::cmd_vel_publish_(double vx, double wz)
{
	geometry_msgs::msg::Twist t;
	t.linear.x = std::clamp(vx, -max_vx_, max_vx_);
	t.angular.z = std::clamp(wz, -max_wz_, max_wz_);
	cmd_vel_publisher_->publish(t);
}

void RobotBehaviourNode::fleet_command_callback_(mini_pupper_interfaces::msg::FleetCommand::ConstSharedPtr msg)
{
	if (!msg) return;
	last_fleet_command_ = std::const_pointer_cast<mini_pupper_interfaces::msg::FleetCommand>(msg);
	last_fleet_ros_stamp_ = msg->stamp;
	last_fleet_recv_st_ = steady_clock_.now();
}

void RobotBehaviourNode::ekf_pose_callback_(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
	if (!msg) return;
	last_ekf_pose_ = std::const_pointer_cast<geometry_msgs::msg::PoseWithCovarianceStamped>(msg);
	last_pose_ros_stamp_ = msg->header.stamp;
}

void RobotBehaviourNode::control_loop_()
{
	// compute dt from steady clock
	const rclcpp::Time now_st = steady_clock_.now();
	double dt = (now_st - last_tick_st_).seconds();
	last_tick_st_ = now_st;

	if (!last_fleet_command_ || !last_ekf_pose_) {
		cmd_vel_publish_(0.0, 0.0);
		return;
	}

	// staleness gate on fleet reference
	const bool fleet_stale = (now_st - last_fleet_recv_st_) >
		rclcpp::Duration::from_seconds(fleet_stale_sec_);
	if (fleet_stale) {
		cmd_vel_publish_(0.0, 0.0);
		mode_ = Mode::Stationary;
		return;
	}

	// references
	const double psi_ref = last_fleet_command_->target_heading;
	const double vx_ref = last_fleet_command_->forward_velocity;
	const double wz_ff = last_fleet_command_->angular_velocity;

	// current yaw from ekf pose
	const auto & q = last_ekf_pose_->pose.pose.orientation;
	tf2::Quaternion tq(q.x, q.y, q.z, q.w);
	double roll, pitch, yaw;
	tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
	double yaw_err = std::remainder(psi_ref - yaw, 2.0 * M_PI);

	// mode decision
	mode_ = decide_mode_(vx_ref);

	// control law
	double vx_cmd = 0.0;
	double wz_cmd = 0.0;

	switch (mode_) {
		case Mode::RotateOnly: {
			// no linear motion pure heading regulation
			vx_cmd = 0.0;
			wz_cmd = wz_ff + Kp_rotate_ * yaw_err;
			break;
		}
		case Mode::MoveAndAlign: {
			// track heading while moving include feed-forward + bias vs linear speed
			vx_cmd = vx_ref;
			const double wz_bias = wz_offset_factor_ * vx_ref;
			wz_cmd = wz_ff + (Kp_move_ * yaw_err) + wz_bias;
			break;
		}
		case Mode::Stationary:
		default: {
			vx_cmd = 0.0;
			wz_cmd = 0.0;
			break;
		}
	}

	cmd_vel_publish_(vx_cmd, wz_cmd);
}