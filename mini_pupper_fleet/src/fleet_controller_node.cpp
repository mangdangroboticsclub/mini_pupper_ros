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

#include <algorithm>   // std::clamp
#include <cmath>       // std::remainder, M_PI
#include "mini_pupper_fleet/fleet_controller_node.hpp"

FleetControllerNode::FleetControllerNode()
: Node("fleet_controller_node")
{
    RCLCPP_INFO(this->get_logger(), "FleetControllerNode has started.");

    auto qos_cmd_vel = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos_cmd_vel,
        std::bind(&FleetControllerNode::cmd_vel_callback_, this, std::placeholders::_1)
    );

    auto qos_fleet_command = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    fleet_command_publisher_ = this->create_publisher<mini_pupper_interfaces::msg::FleetCommand>("/fleet_command", qos_fleet_command);

    // initialise state
    target_heading_ = 0.0;
    last_linear_x_  = 0.0;
    last_angular_z_ = 0.0;

    // use steady time for both dt and staleness tracking
    last_timer_time_ = steady_clock_.now();
    last_cmd_vel_time_st_ = last_timer_time_;

    // fixed-rate timer loop
    fleet_command_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(FleetCommandPeriod),  // 15 ms => 66.67 hz
        std::bind(&FleetControllerNode::fleet_command_loop_, this)
    );
}

void FleetControllerNode::cmd_vel_callback_(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    // check pointer to be safe
    if (!msg) {
        return;
    }

    // store latest command and when it arrived (steady time)
    last_linear_x_ = msg->linear.x;
    last_angular_z_ = msg->angular.z;
    last_cmd_vel_time_st_ = steady_clock_.now();
}

void FleetControllerNode::fleet_command_loop_()
{
    // calculate dt from steady clock
    const rclcpp::Time now_st = steady_clock_.now();

    double dt = (now_st - last_timer_time_).seconds();
    last_timer_time_ = now_st;

    // clamp dt to avoid large jumps
    const double tick = static_cast<double>(FleetCommandPeriod) / 1000.0;
    dt = std::clamp(dt, 0.0, 2.0 * tick);

    // check for stale command using steady time and rclcpp::Duration
    const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(CmdVelStaleSec);
    bool stale = (now_st - last_cmd_vel_time_st_) > timeout;

    double vx;
    double wz;
    if (stale) {
        vx = 0.0;
        wz = 0.0;
    } else {
        vx = last_linear_x_;
        wz = last_angular_z_;
    }

    // integrate heading and wrap to [-pi, pi]
    target_heading_ = std::remainder(target_heading_ + wz * dt, 2.0 * M_PI);

    // publish fleet command
    mini_pupper_interfaces::msg::FleetCommand fleet_command_msg;
    fleet_command_msg.stamp = this->now();
    fleet_command_msg.target_heading   = target_heading_;
    fleet_command_msg.forward_velocity = vx;
    fleet_command_msg.angular_velocity = wz;

    RCLCPP_INFO(this->get_logger(),
                 "dt=%.3f stale=%d heading=%.2f vx=%.2f wz=%.2f",
                 dt, stale,
                 fleet_command_msg.target_heading,
                 fleet_command_msg.forward_velocity,
                 fleet_command_msg.angular_velocity);

    fleet_command_publisher_->publish(fleet_command_msg);
}
