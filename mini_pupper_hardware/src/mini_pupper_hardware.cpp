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

#include "mini_pupper_hardware/mini_pupper_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mini_pupper_hardware/esp32_interface.hpp"

namespace mini_pupper_hardware
{
namespace
{
constexpr std::array<const char *, 12> kHardwareJointOrder = {
  "base_rf1", "rf1_rf2", "rf2_rf3",
  "base_lf1", "lf1_lf2", "lf2_lf3",
  "base_rb1", "rb1_rb2", "rb2_rb3",
  "base_lb1", "lb1_lb2", "lb2_lb3",
};
}  // namespace

CallbackReturn MiniPupperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Initializing Mini Pupper Hardware");

  // Extract joint names from URDF (info_.joints gives us the ros2_control order)
  joint_names_.clear();
  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  if (joint_names_.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("MiniPupperHardware"),
      "Expected %zu joints, got %zu from URDF", NUM_JOINTS, joint_names_.size());
    return CallbackReturn::ERROR;
  }

  // Build mapping from hardware servo order back into the ros2_control joint array.
  if (!build_joint_mapping())
  {
    return CallbackReturn::ERROR;
  }

  // Get configuration from hardware parameters
  if (info_.hardware_parameters.count("hardware_interface_type"))
  {
    hardware_interface_type_ = info_.hardware_parameters.at("hardware_interface_type");
  } else
  {
    hardware_interface_type_ = "mock";
  }

  if (hardware_interface_type_ == "mock")
  {
    use_mock_hardware_ = true;
    RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Using mock hardware interface");
  } else
  {
    use_mock_hardware_ = false;
    RCLCPP_INFO(
      rclcpp::get_logger("MiniPupperHardware"),
      "Using hardware interface type: %s", hardware_interface_type_.c_str());
  }

  initialize_state_storage();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MiniPupperHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Configuring Mini Pupper Hardware");

  // Initialize ESP32 interface if not using mock hardware
  if (!use_mock_hardware_)
  {
    try
    {
      esp32_interface_ = std::make_unique<ESP32Interface>();
      if (!esp32_interface_->is_connected())
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("MiniPupperHardware"),
          "Failed to connect to ESP32 proxy");
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("MiniPupperHardware"), "ESP32 interface configured");
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("MiniPupperHardware"),
        "Failed to configure ESP32 interface: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MiniPupperHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Activating Mini Pupper Hardware");

  // Best-effort: read once on activation so we don't immediately command zeros
  // before controllers publish a valid posture.
  if (!use_mock_hardware_)
  {
    read_state_from_hardware();
  }

  read_counter_ = 0;

  // Initialize position commands with current positions
  hw_position_commands_ = hw_positions_;

  RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Hardware activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MiniPupperHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Deactivating Mini Pupper Hardware");

  if (!use_mock_hardware_)
  {
    // Deactivate hardware if not using mock
    RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Hardware deactivated");
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MiniPupperHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  update_velocities(period);

  if (!use_mock_hardware_)
  {
    // Only read from hardware every 10 cycles (10Hz instead of 100Hz)
    // to avoid blocking the control loop
    if (++read_counter_ >= 10)
    {
      read_state_from_hardware();
      read_counter_ = 0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MiniPupperHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_mock_hardware_)
  {
    // In mock mode, simply mirror commands to state
    for (size_t i = 0; i < NUM_JOINTS; ++i)
    {
      hw_positions_[i] = hw_position_commands_[i];
    }
  } else
  {
    // Send commands but don't block if it fails
    // The control loop must continue even if communication fails temporarily
    send_commands_to_hardware();
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MiniPupperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MiniPupperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
  }

  return command_interfaces;
}

void MiniPupperHardware::initialize_state_storage()
{
  hw_positions_.assign(NUM_JOINTS, 0.0);
  hw_velocities_.assign(NUM_JOINTS, 0.0);
  hw_efforts_.assign(NUM_JOINTS, 0.0);

  hw_position_commands_.assign(NUM_JOINTS, 0.0);

  hw_positions_prev_.assign(NUM_JOINTS, 0.0);
}

void MiniPupperHardware::update_velocities(const rclcpp::Duration & period)
{
  if (period.seconds() > 0.0)
  {
    for (size_t i = 0; i < NUM_JOINTS; ++i)
    {
      hw_velocities_[i] = (hw_positions_[i] - hw_positions_prev_[i]) / period.seconds();
      hw_positions_prev_[i] = hw_positions_[i];
    }
  }
}

void MiniPupperHardware::send_commands_to_hardware()
{
  if (!esp32_interface_ || !esp32_interface_->is_connected())
  {
    // Don't spam errors - just skip this cycle
    return;
  }

  // Convert joint targets to calibrated servo positions in hardware servo order:
  // RF, LF, RB, LB x (abduction, hip, knee_abs).
  std::array<uint16_t, ESP32Interface::NUM_SERVOS> servo_positions;
  servo_positions.fill(static_cast<uint16_t>(NEUTRAL_POSITION));

  for (size_t servo_index = 0; servo_index < servo_positions.size(); ++servo_index)
  {
    const size_t joint_index = hardware_joint_to_urdf_index_[servo_index];
    const size_t axis_index = servo_index % 3;
    const size_t leg_index = servo_index / 3;
    servo_positions[servo_index] = angle_to_servo_position(
      hw_position_commands_[joint_index], axis_index, leg_index);
  }

  // Send to hardware - don't care if it fails, we'll try again next cycle
  esp32_interface_->servos_set_position(servo_positions);
}

void MiniPupperHardware::read_state_from_hardware()
{
  if (!esp32_interface_ || !esp32_interface_->is_connected())
  {
    // Don't spam errors - just skip this read
    return;
  }

  // Read current servo positions from hardware
  auto servo_positions = esp32_interface_->servos_get_position();

  if (servo_positions.size() != NUM_JOINTS)
  {
    // Failed to read - keep previous values
    return;
  }

  for (size_t servo_index = 0; servo_index < servo_positions.size(); ++servo_index)
  {
    const size_t joint_index = hardware_joint_to_urdf_index_[servo_index];
    const size_t axis_index = servo_index % 3;
    const size_t leg_index = servo_index / 3;
    hw_positions_[joint_index] = servo_position_to_angle(
      servo_positions[servo_index], axis_index, leg_index);
  }
}

uint16_t MiniPupperHardware::angle_to_servo_position(
  double angle_rad, size_t axis_index, size_t leg_index)
{
  if (axis_index >= 3 || leg_index >= 4)
  {
    return static_cast<uint16_t>(NEUTRAL_POSITION);
  }

  const double neutral_angle = NEUTRAL_ANGLES_RAD[axis_index];
  const int multiplier = SERVO_MULTIPLIERS[axis_index][leg_index];

  // Mirrors MangDang.mini_pupper.HardwareInterface.angle_to_position
  const double angle_deviation = (angle_rad - neutral_angle) * static_cast<double>(multiplier);
  double servo_position = NEUTRAL_POSITION - MICROS_PER_RAD * angle_deviation;

  // Check for NaN (matches Python behavior)
  if (std::isnan(servo_position))
  {
    return 0;
  }

  servo_position = std::max(0.0, std::min(1023.0, servo_position));
  return static_cast<uint16_t>(std::lround(servo_position));
}

double MiniPupperHardware::servo_position_to_angle(
  uint16_t servo_position, size_t axis_index, size_t leg_index)
{
  if (axis_index >= 3 || leg_index >= 4)
  {
    return 0.0;
  }

  const double neutral_angle = NEUTRAL_ANGLES_RAD[axis_index];
  const int multiplier = SERVO_MULTIPLIERS[axis_index][leg_index];
  if (multiplier == 0)
  {
    return neutral_angle;
  }

  // Invert: servo_position = neutral - micros_per_rad *
  // ((angle - neutral_angle) * multiplier)
  const double delta = (NEUTRAL_POSITION - static_cast<double>(servo_position)) / MICROS_PER_RAD;
  return neutral_angle + (delta / static_cast<double>(multiplier));
}

bool MiniPupperHardware::build_joint_mapping()
{
  auto logger = rclcpp::get_logger("MiniPupperHardware");

  for (size_t servo_index = 0; servo_index < kHardwareJointOrder.size(); ++servo_index)
  {
    const auto joint_it = std::find(
      joint_names_.begin(), joint_names_.end(), kHardwareJointOrder[servo_index]);
    if (joint_it == joint_names_.end())
    {
      std::ostringstream expected_order;
      for (size_t i = 0; i < kHardwareJointOrder.size(); ++i)
      {
        if (i > 0)
        {
          expected_order << ", ";
        }
        expected_order << kHardwareJointOrder[i];
      }

      std::ostringstream actual_order;
      for (size_t i = 0; i < joint_names_.size(); ++i)
      {
        if (i > 0)
        {
          actual_order << ", ";
        }
        actual_order << joint_names_[i];
      }

      RCLCPP_ERROR(
        logger,
        "URDF joint list is missing expected joint '%s'. Expected joints: [%s]. Actual joints: [%s]",
        kHardwareJointOrder[servo_index], expected_order.str().c_str(), actual_order.str().c_str());
      return false;
    }

    hardware_joint_to_urdf_index_[servo_index] =
      static_cast<size_t>(std::distance(joint_names_.begin(), joint_it));
  }

  std::ostringstream mapping;
  for (size_t servo_index = 0; servo_index < kHardwareJointOrder.size(); ++servo_index)
  {
    if (servo_index > 0)
    {
      mapping << ", ";
    }
    mapping << kHardwareJointOrder[servo_index] << "->" << hardware_joint_to_urdf_index_[servo_index];
  }

  RCLCPP_INFO(logger, "Validated hardware joint mapping: %s", mapping.str().c_str());
  return true;
}

}  // namespace mini_pupper_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mini_pupper_hardware::MiniPupperHardware, hardware_interface::SystemInterface)
