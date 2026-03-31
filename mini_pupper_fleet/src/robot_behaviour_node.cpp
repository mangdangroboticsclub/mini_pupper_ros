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
  fleet_command_subscription_ =
    this->create_subscription<mini_pupper_interfaces::msg::FleetCommand>(
    "/fleet_command", qos_fleet_command,
    std::bind(&RobotBehaviourNode::fleet_command_callback_, this, std::placeholders::_1));

  auto qos_ekf_pose = rclcpp::SensorDataQoS();
  ekf_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose", qos_ekf_pose,
    std::bind(&RobotBehaviourNode::ekf_pose_callback_, this, std::placeholders::_1));

  robot_command_publisher_ = this->create_publisher<mini_pupper_interfaces::msg::Command>(
    "robot_command", 10);
}

RobotBehaviourNode::Mode RobotBehaviourNode::decide_mode_(double vx_ref) const
{
  if (std::abs(vx_ref) < 1e-6) {
    return Mode::RotateOnly;
  }
  return Mode::MoveAndAlign;
}

bool RobotBehaviourNode::vel_zero_(const std::vector<double> & vel, double yaw_rate)
{
  /**
   * Returns true if both horizontal velocity and yaw_rate are approximately zero.
   */
  const double tolerance = 1e-3;

  return (std::abs(vel[0]) <= tolerance) &&
         (std::abs(vel[1]) <= tolerance) &&
         (std::abs(yaw_rate) <= tolerance);
}

mini_pupper_interfaces::msg::Command RobotBehaviourNode::create_command_(
  const std::vector<double> & vel,
  double yaw_rate,
  double pitch)
{
  mini_pupper_interfaces::msg::Command cmd;

  // Set height
  cmd.height = config_.default_z_ref;

  // Set default standing locations
  mini_pupper_interfaces::msg::Matrix3x4 legs_location;
  std::copy(
    config_.default_stance[0].begin(),
    config_.default_stance[0].end(), legs_location.row1.begin());
  std::copy(
    config_.default_stance[1].begin(),
    config_.default_stance[1].end(), legs_location.row2.begin());
  std::copy(
    config_.default_stance[2].begin(),
    config_.default_stance[2].end(), legs_location.row3.begin());
  cmd.legs_location = legs_location;

  // Clamp velocity components
  double x_vel = std::clamp(
    vel[0],
    -config_.max_x_velocity,
    config_.max_x_velocity
  );

  double y_vel = std::clamp(
    vel[1],
    -config_.max_y_velocity,
    config_.max_y_velocity
  );

  // Clamp yaw rate
  double yaw_rate_clipped = std::clamp(
    yaw_rate,
    -config_.max_yaw_rate,
    config_.max_yaw_rate
  );

  // Clamp pitch (±π/10 radians ≈ ±18 degrees)
  double pitch_clipped = std::clamp(
    pitch,
    -M_PI / 10.0,
    M_PI / 10.0
  );

  // Set command values
  cmd.horizontal_velocity = {x_vel, y_vel};
  cmd.yaw_rate = yaw_rate_clipped;
  cmd.roll = 0.0;
  cmd.pitch = pitch_clipped;
  cmd.yaw = 0.0;        // Only used when robot is stationary

  // Detect zero↔non-zero edge and fire trot_event only once
  bool is_zero = vel_zero_(vel, yaw_rate);
  cmd.trot_event = (prev_zero_ != is_zero);
  prev_zero_ = is_zero;

  return cmd;
}

void RobotBehaviourNode::robot_command_publish_(double vx, double wz)
{
  std::vector<double> vel = {vx, 0.0};
  auto cmd = create_command_(vel, wz, 0.0);
  robot_command_publisher_->publish(cmd);
}

void RobotBehaviourNode::fleet_command_callback_(
  mini_pupper_interfaces::msg::FleetCommand::ConstSharedPtr msg)
{
  if (!msg) {return;}
  last_fleet_command_ = std::const_pointer_cast<mini_pupper_interfaces::msg::FleetCommand>(msg);
  last_fleet_ros_stamp_ = msg->stamp;
  last_fleet_recv_st_ = steady_clock_.now();
}

void RobotBehaviourNode::ekf_pose_callback_(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!msg) {return;}
  last_ekf_pose_ = std::const_pointer_cast<geometry_msgs::msg::PoseWithCovarianceStamped>(msg);
  last_pose_ros_stamp_ = msg->header.stamp;
}

void RobotBehaviourNode::control_loop_()
{
  // compute dt from steady clock
  const rclcpp::Time now_st = steady_clock_.now();
  double dt = (now_st - last_tick_st_).seconds();
  (void)dt;
  last_tick_st_ = now_st;

  if (!last_fleet_command_ || !last_ekf_pose_) {
    robot_command_publish_(0.0, 0.0);
    return;
  }

  // staleness gate on fleet reference
  const bool fleet_stale = (now_st - last_fleet_recv_st_) >
    rclcpp::Duration::from_seconds(fleet_stale_sec_);
  if (fleet_stale) {
    robot_command_publish_(0.0, 0.0);
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

  robot_command_publish_(vx_cmd, wz_cmd);
}
