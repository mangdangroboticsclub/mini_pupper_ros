// SPDX-License-Identifier: Apache-2.0
//
// Copyright (c) 2024 Mini Pupper Contributors
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mini_pupper_hardware/esp32_interface.hpp"

namespace mini_pupper_hardware
{
CallbackReturn MiniPupperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Initializing Mini Pupper Hardware");

  // Get configuration from hardware parameters
  if (info_.hardware_parameters.count("hardware_interface_type"))
  {
    hardware_interface_type_ = info_.hardware_parameters.at("hardware_interface_type");
  }
  else
  {
    hardware_interface_type_ = "mock";
  }

  if (hardware_interface_type_ == "mock")
  {
    use_mock_hardware_ = true;
    RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Using mock hardware interface");
  }
  else
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

  // Initialize position commands with current positions
  hw_position_commands_ = hw_positions_;

  if (!use_mock_hardware_)
  {
    // Initialize hardware if not using mock
    RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Hardware activated");
  }
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
    static int read_counter = 0;
    if (++read_counter >= 10)
    {
      read_state_from_hardware();
      read_counter = 0;
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
  }
  else
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
  hw_velocity_commands_.assign(NUM_JOINTS, 0.0);
  hw_effort_commands_.assign(NUM_JOINTS, 0.0);

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
  if// Don't spam errors - just skip this cycle
    return;
  }

  // Convert radians to servo raw values (0-1023)
  // Map from URDF joint order to hardware servo order
  std::array<uint16_t, ESP32Interface::NUM_SERVOS> servo_positions;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    size_t servo_index = joint_to_servo_map_[i];
    servo_positions[servo_index] = radians_to_servo(hw_position_commands_[i]);
  }

  // Send to hardware - don't care if it fails, we'll try again next cycle
  esp32_interface_->servos_set_position(servo_positions);   rclcpp::get_logger("MiniPupperHardware"), steady_clock_, 1000,
      "Failed to send servo commands");
  }
}

void MiniPupperHardware::read_state_from_hardware()
{
  if (!esp32_interface_ || !esp32_interface_->is_connected())
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("MiniPupperHardware"), steady_clock_, 1000,
      "ESP32 interface not connected");
    return;
  }

  // Read current servo positions from hardware
  au// Don't spam errors - just skip this read
    return;
  }

  // Read current servo positions from hardware
  auto servo_positions = esp32_interface_->servos_get_position();

  if (servo_positions.size() != NUM_JOINTS)
  {
    // Failed to read - keep previous values
    size_t servo_index = joint_to_servo_map_[i];
    hw_positions_[i] = servo_to_radians(servo_positions[servo_index]);
  }
}

uint16_t MiniPupperHardware::radians_to_servo(double radians)
{
  // Clamp radians to [-π, π]
  double clamped = std::max(-M_PI, std::min(M_PI, radians));
  // Convert to servo range [0, 1023], neutral position 512
  uint16_t servo_value = static_cast<uint16_t>(512.0 + clamped * RAD_TO_SERVO);
  return std::max(uint16_t(0), std::min(uint16_t(1023), servo_value));
}

double MiniPupperHardware::servo_to_radians(uint16_t servo_value)
{
  // Convert from servo range [0, 1023] to radians
  // Neutral position 512 maps to 0 radians
  double offset = static_cast<double>(servo_value) - 512.0;
  return offset * SERVO_TO_RAD;
}

}  // namespace mini_pupper_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mini_pupper_hardware::MiniPupperHardware, hardware_interface::SystemInterface)
