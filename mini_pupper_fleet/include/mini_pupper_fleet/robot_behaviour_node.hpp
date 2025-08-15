#ifndef ROBOT_BEHAVIOUR_NODE_HPP_
#define ROBOT_BEHAVIOUR_NODE_HPP_

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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mini_pupper_interfaces/msg/fleet_command.hpp>

class RobotBehaviourNode : public rclcpp::Node
{
public:
  RobotBehaviourNode();

private:
  // control timing
  static constexpr int ControlPeriodMs = 15;  // 66.7 hz
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_tick_st_{};

  // io
  rclcpp::Subscription<mini_pupper_interfaces::msg::FleetCommand>::SharedPtr fleet_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // latest inputs
  mini_pupper_interfaces::msg::FleetCommand::SharedPtr last_fleet_command_;
  rclcpp::Time last_fleet_ros_stamp_{};
  rclcpp::Time last_fleet_recv_st_{};
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_ekf_pose_;
  rclcpp::Time last_pose_ros_stamp_{};

  // behaviour state
  enum class Mode : uint8_t {
    Stationary = 0,
    RotateOnly = 1,
    MoveAndAlign = 2
  };
  Mode mode_{Mode::Stationary};

  // gains and limits
  double Kp_move_ = 1.0;        // p-gain when moving
  double Kp_rotate_ = 1.0;      // p-gain when rotating in place

  // angular rate bias applied only when moving: wz_offset = wz_offset_factor_ * vx_ref
  double wz_offset_factor_ = 0.5;  // rad/s per m/s

  double max_wz_ = 1.5;         // rad/s
  double max_vx_ = 0.5;         // m/s

  // staleness (fleet only)
  double fleet_stale_sec_ = 0.25;

  // callbacks
  void fleet_command_callback_(mini_pupper_interfaces::msg::FleetCommand::ConstSharedPtr msg);
  void ekf_pose_callback_(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void control_loop_();

  // helpers
  static double wrap_to_pi_(double a);
  static double yaw_from_pose_(const geometry_msgs::msg::PoseWithCovarianceStamped & m);
  Mode decide_mode_(double vx_ref) const;
  void cmd_vel_publish_(double vx, double wz);
};

#endif
