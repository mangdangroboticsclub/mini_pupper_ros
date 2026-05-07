// SPDX-License-Identifier: Apache-2.0
//
// Copyright (c) 2026 MangDang
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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mini_pupper_controllers
{
class SimpleQuadrupedController : public controller_interface::ControllerInterface
{
public:
  using CommandMsg = std_msgs::msg::Float64MultiArray;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void assign_default_if_needed();
  std::vector<double> apply_linkage_compensation(const std::vector<double> & positions) const;

  std::vector<std::string> joint_names_;
  std::vector<double> default_positions_;
  std::vector<double> commanded_positions_;
  std::vector<double> idle_start_positions_;

  rclcpp::Subscription<CommandMsg>::SharedPtr command_subscription_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CommandMsg>> command_buffer_;
  bool has_external_command_{false};
  bool parallel_linkage_compensation_{false};
  bool idle_ramp_enabled_{false};
  bool idle_ramp_started_{false};
  double idle_hold_duration_sec_{0.0};
  double idle_ramp_duration_sec_{0.0};
  double idle_ramp_elapsed_sec_{0.0};
};
}  // namespace mini_pupper_controllers
