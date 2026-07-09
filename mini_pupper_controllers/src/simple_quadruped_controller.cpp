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

#include "mini_pupper_controllers/simple_quadruped_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace mini_pupper_controllers
{
controller_interface::CallbackReturn SimpleQuadrupedController::on_init()
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SimpleQuadrupedController"), "Unable to access node interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!node->has_parameter("joints")) {
    node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  }
  if (!node->has_parameter("default_positions")) {
    node->declare_parameter<std::vector<double>>("default_positions", std::vector<double>{});
  }
  if (!node->has_parameter("parallel_linkage_compensation")) {
    node->declare_parameter<bool>("parallel_linkage_compensation", false);
  }
  if (!node->has_parameter("idle_ramp_enabled")) {
    node->declare_parameter<bool>("idle_ramp_enabled", false);
  }
  if (!node->has_parameter("idle_hold_duration_sec")) {
    node->declare_parameter<double>("idle_hold_duration_sec", 0.0);
  }
  if (!node->has_parameter("idle_ramp_duration_sec")) {
    node->declare_parameter<double>("idle_ramp_duration_sec", 0.0);
  }
  command_buffer_.writeFromNonRT(nullptr);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SimpleQuadrupedController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());

  for (const auto & joint : joint_names_) {
    config.names.emplace_back(joint + "/position");
  }

  return config;
}

controller_interface::InterfaceConfiguration
SimpleQuadrupedController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());

  for (const auto & joint : joint_names_) {
    config.names.emplace_back(joint + "/position");
  }

  return config;
}

controller_interface::CallbackReturn SimpleQuadrupedController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SimpleQuadrupedController"), "Unable to access node interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_ = node->get_parameter("joints").as_string_array();
  default_positions_ = node->get_parameter("default_positions").as_double_array();
  parallel_linkage_compensation_ = node->get_parameter("parallel_linkage_compensation").as_bool();
  idle_ramp_enabled_ = node->get_parameter("idle_ramp_enabled").as_bool();
  idle_hold_duration_sec_ = node->get_parameter("idle_hold_duration_sec").as_double();
  idle_ramp_duration_sec_ = node->get_parameter("idle_ramp_duration_sec").as_double();

  RCLCPP_INFO(
    node->get_logger(), "Parallel linkage compensation: %s",
    parallel_linkage_compensation_ ? "ENABLED (simulation)" : "disabled (hardware)");
  RCLCPP_INFO(
    node->get_logger(), "Idle ramp: %s (hold %.2fs, ramp %.2fs)",
    idle_ramp_enabled_ ? "enabled" : "disabled", idle_hold_duration_sec_, idle_ramp_duration_sec_);
  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!default_positions_.empty() && default_positions_.size() != joint_names_.size()) {
    RCLCPP_WARN(
      node->get_logger(),
      "Parameter 'default_positions' size (%zu) does not match joints size (%zu);"
      " padding with zeros.",
      default_positions_.size(), joint_names_.size());
  }

  assign_default_if_needed();
  commanded_positions_ = default_positions_;
  idle_start_positions_ = default_positions_;
  has_external_command_ = false;
  idle_ramp_started_ = false;

  // Log the default positions being used
  RCLCPP_INFO(node->get_logger(), "Controller initialized with default_positions:");
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    RCLCPP_INFO(
      node->get_logger(), "  %s: %.4f rad (%.2f deg)",
      joint_names_[i].c_str(), default_positions_[i],
      default_positions_[i] * 180.0 / M_PI);
  }

  command_buffer_.writeFromNonRT(nullptr);

  command_subscription_ = node->create_subscription<CommandMsg>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CommandMsg::SharedPtr msg) {command_buffer_.writeFromNonRT(msg);});

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleQuadrupedController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  assign_default_if_needed();
  commanded_positions_ = default_positions_;
  idle_start_positions_ = default_positions_;
  has_external_command_ = false;
  idle_ramp_started_ = false;
  idle_ramp_elapsed_sec_ = 0.0;
  command_buffer_.writeFromNonRT(nullptr);

  if (command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Number of command interfaces (%zu) does not match joints (%zu)",
      command_interfaces_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() >= joint_names_.size()) {
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      std::optional<double> pos_opt = state_interfaces_[index].get_optional();
      if (pos_opt) {
        const double pos = pos_opt.value();
        if (!std::isnan(pos)) {
          commanded_positions_[index] = pos;
          idle_start_positions_[index] = pos;
        }
      }
    }
  }

  for (size_t index = 0; index < command_interfaces_.size(); ++index) {
    command_interfaces_[index].set_value(commanded_positions_[index]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleQuadrupedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  commanded_positions_.clear();
  idle_start_positions_.clear();
  has_external_command_ = false;
  idle_ramp_started_ = false;
  idle_ramp_elapsed_sec_ = 0.0;
  command_buffer_.writeFromNonRT(nullptr);
  command_subscription_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SimpleQuadrupedController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto command_ptr = command_buffer_.readFromRT();
  if (command_ptr && *command_ptr) {
    const auto & msg = *(*command_ptr);
    if (msg.data.size() == joint_names_.size()) {
      commanded_positions_.assign(msg.data.begin(), msg.data.end());
      has_external_command_ = true;
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "Command size (%zu) does not match joints (%zu)",
        msg.data.size(), joint_names_.size());
    }
  }

  std::vector<double> target_positions;
  if (!has_external_command_) {
    if (
      idle_ramp_enabled_ && idle_ramp_duration_sec_ > 0.0 &&
      idle_start_positions_.size() == default_positions_.size())
    {
      idle_ramp_elapsed_sec_ += period.seconds();
      target_positions.resize(default_positions_.size(), 0.0);

      if (idle_ramp_elapsed_sec_ <= idle_hold_duration_sec_) {
        for (size_t index = 0; index < default_positions_.size(); ++index) {
          std::optional<double> pos_opt = state_interfaces_[index].get_optional();
          const double pos = pos_opt ? pos_opt.value() : std::numeric_limits<double>::quiet_NaN();
          target_positions[index] = std::isnan(pos) ? commanded_positions_[index] : pos;
        }
        idle_start_positions_ = target_positions;
        idle_ramp_started_ = false;
      } else {
        if (!idle_ramp_started_) {
          for (size_t index = 0; index < default_positions_.size(); ++index) {
            std::optional<double> pos_opt = state_interfaces_[index].get_optional();
            const double pos = pos_opt ? pos_opt.value() : std::numeric_limits<double>::quiet_NaN();
            idle_start_positions_[index] = std::isnan(pos) ? commanded_positions_[index] : pos;
          }
          idle_ramp_started_ = true;
        }

        const double ramp_time_sec = idle_ramp_elapsed_sec_ - idle_hold_duration_sec_;
        const double raw_alpha = std::clamp(ramp_time_sec / idle_ramp_duration_sec_, 0.0, 1.0);
        const double alpha = raw_alpha * raw_alpha * (3.0 - 2.0 * raw_alpha);

        for (size_t index = 0; index < default_positions_.size(); ++index) {
          target_positions[index] =
            idle_start_positions_[index] +
            (default_positions_[index] - idle_start_positions_[index]) * alpha;
        }
      }
    } else {
      target_positions = commanded_positions_;
    }
  } else {
    // External commands come from the Stanford controller in hardware-space and
    // need the simulation-specific mapping before they can be sent to Gazebo.
    target_positions = parallel_linkage_compensation_ ?
      apply_linkage_compensation(commanded_positions_) :
      commanded_positions_;
  }

  if (command_interfaces_.size() != target_positions.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Mismatch between command interfaces (%zu) and target positions (%zu)",
      command_interfaces_.size(), target_positions.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t index = 0; index < command_interfaces_.size(); ++index) {
    command_interfaces_[index].set_value(target_positions[index]);
  }

  return controller_interface::return_type::OK;
}

std::vector<double> SimpleQuadrupedController::apply_linkage_compensation(
  const std::vector<double> & positions) const
{
  // On real hardware the knee servo is body-frame-referenced via a parallel 4-bar linkage:
  //   physical_knee_angle = hip_angle + knee_angle
  // The URDF models lf2_lf3 as a serial joint relative to the thigh, so Gazebo expects:
  //   urdf_knee = knee_cmd - hip_cmd
  // Joint layout per leg (stride = 3): [abduction, hip, knee]
  std::vector<double> compensated = positions;
  const size_t stride = 3;
  const size_t num_legs = positions.size() / stride;
  for (size_t leg = 0; leg < num_legs; ++leg) {
    const size_t hip_idx = leg * stride + 1;
    const size_t knee_idx = leg * stride + 2;
    compensated[knee_idx] = positions[knee_idx] - positions[hip_idx];
  }
  return compensated;
}

void SimpleQuadrupedController::assign_default_if_needed()
{
  auto node = get_node();
  if (default_positions_.size() != joint_names_.size()) {
    default_positions_.assign(joint_names_.size(), 0.0);
  }

  if (commanded_positions_.size() != joint_names_.size()) {
    commanded_positions_.assign(joint_names_.size(), 0.0);
  }

  if (idle_start_positions_.size() != joint_names_.size()) {
    idle_start_positions_.assign(joint_names_.size(), 0.0);
  }
}

}  // namespace mini_pupper_controllers

PLUGINLIB_EXPORT_CLASS(
  mini_pupper_controllers::SimpleQuadrupedController, controller_interface::ControllerInterface)
