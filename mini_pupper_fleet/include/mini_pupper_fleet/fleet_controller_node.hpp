#ifndef FLEET_CONTROLLER_NODE_HPP_
#define FLEET_CONTROLLER_NODE_HPP_

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
#include <mini_pupper_interfaces/msg/fleet_command.hpp>

class FleetControllerNode : public rclcpp::Node
{
public:
    FleetControllerNode();

private:
    // subscription for cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    void cmd_vel_callback_(geometry_msgs::msg::Twist::ConstSharedPtr msg);

    // clocks and timing (use steady time for both dt and staleness)
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time last_timer_time_;
    rclcpp::Time last_cmd_vel_time_st_;

    // publisher for fleet_command
    rclcpp::Publisher<mini_pupper_interfaces::msg::FleetCommand>::SharedPtr fleet_command_publisher_;

    // fixed-rate timer loop
    rclcpp::TimerBase::SharedPtr fleet_command_timer_;
    static constexpr int FleetCommandPeriod = 15;   // 0.015 s (~66.67 hz)
    static constexpr double CmdVelStaleSec = 0.5;  // auto-stop if no cmd_vel for this long
    void fleet_command_loop_();

    // state
    double target_heading_;
    double last_linear_x_;
    double last_angular_z_;
};

#endif

