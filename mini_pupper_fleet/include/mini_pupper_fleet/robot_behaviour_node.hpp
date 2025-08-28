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
#include <mini_pupper_interfaces/msg/command.hpp>
#include <mini_pupper_interfaces/msg/matrix3x4.hpp>
#include <vector>

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

  rclcpp::Subscription<mini_pupper_interfaces::msg::FleetCommand>::SharedPtr
    fleet_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ekf_pose_subscription_;
  rclcpp::Publisher<mini_pupper_interfaces::msg::Command>::SharedPtr robot_command_publisher_;

  // latest inputs
  mini_pupper_interfaces::msg::FleetCommand::SharedPtr last_fleet_command_;
  rclcpp::Time last_fleet_ros_stamp_{};
  rclcpp::Time last_fleet_recv_st_{};
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_ekf_pose_;
  rclcpp::Time last_pose_ros_stamp_{};

  // behaviour state
  enum class Mode : uint8_t
  {
    Stationary = 0,
    RotateOnly = 1,
    MoveAndAlign = 2
  };
  Mode mode_{Mode::Stationary};

  // gains and limits
  double Kp_move_ = 1.0; // p-gain when moving
  double Kp_rotate_ = 0.0; // p-gain when rotating in place

  // angular rate bias applied only when moving: wz_offset = wz_offset_factor_ * vx_ref
  double wz_offset_factor_ = -0.33; // rad/s per m/s

  double max_wz_ = 1.0;
  double max_vx_ = 0.5;

  // staleness (fleet only)
  double fleet_stale_sec_ = 0.25;

  struct Config
  {
    double default_z_ref = -0.07;
    double max_x_velocity = 0.20;
    double max_y_velocity = 0.20;
    double max_yaw_rate = 2.0;
    double delta_x = 0.059;
    double delta_y = 0.050;
    double x_shift = 0.0;
    double z_shift = 0.0;

    std::vector<std::vector<double>> default_stance = {
      {
        delta_x + x_shift,
        delta_x + x_shift,
        -delta_x + x_shift,
        -delta_x + x_shift
      },
      {-delta_y, delta_y, -delta_y, delta_y},
      {z_shift, z_shift, z_shift, z_shift}
    };
  } config_;

  // State tracking for trot events
  bool prev_zero_ = true;

  // callbacks
  void fleet_command_callback_(mini_pupper_interfaces::msg::FleetCommand::ConstSharedPtr msg);
  void ekf_pose_callback_(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void control_loop_();

  // helpers
  Mode decide_mode_(double vx_ref) const;
  void robot_command_publish_(double vx, double wz);

  // Command creation helpers
  bool vel_zero_(const std::vector<double> & vel, double yaw_rate);
  mini_pupper_interfaces::msg::Command create_command_(
    const std::vector<double> & vel = {0.0, 0.0},
    double yaw_rate = 0.0,
    double pitch = 0.0
  );
};

#endif
