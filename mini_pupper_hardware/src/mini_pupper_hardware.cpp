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

  // Log the joint order we received
  RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Joint order from ros2_control:");
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "  [%zu] %s", i, joint_names_[i].c_str());
  }

  // Build mapping from URDF joint order to our canonical order (LF, RF, LB, RB)
  build_joint_mapping();

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
      
      // Debug: log expected neutral servo positions
      RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Expected neutral servo positions:");
      RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Neutral angles: [%.3f, %.3f, %.3f] rad = [%.1f, %.1f, %.1f] deg",
                  NEUTRAL_ANGLES_RAD[0], NEUTRAL_ANGLES_RAD[1], NEUTRAL_ANGLES_RAD[2],
                  NEUTRAL_ANGLES_RAD[0] * 180.0 / M_PI, NEUTRAL_ANGLES_RAD[1] * 180.0 / M_PI, NEUTRAL_ANGLES_RAD[2] * 180.0 / M_PI);
      RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Servo multipliers:");
      RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "  axis 0 (abd):  [%2d, %2d, %2d, %2d]", 
                  SERVO_MULTIPLIERS[0][0], SERVO_MULTIPLIERS[0][1], SERVO_MULTIPLIERS[0][2], SERVO_MULTIPLIERS[0][3]);
      RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "  axis 1 (hip):  [%2d, %2d, %2d, %2d]", 
                  SERVO_MULTIPLIERS[1][0], SERVO_MULTIPLIERS[1][1], SERVO_MULTIPLIERS[1][2], SERVO_MULTIPLIERS[1][3]);
      RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "  axis 2 (knee): [%2d, %2d, %2d, %2d]", 
                  SERVO_MULTIPLIERS[2][0], SERVO_MULTIPLIERS[2][1], SERVO_MULTIPLIERS[2][2], SERVO_MULTIPLIERS[2][3]);
      
      for (size_t leg = 0; leg < 4; leg++)
      {
        const char* leg_names[] = {"RF", "LF", "RB", "LB"};
        uint16_t abd_neutral = angle_to_servo_position(NEUTRAL_ANGLES_RAD[0], 0, leg);
        uint16_t hip_neutral = angle_to_servo_position(NEUTRAL_ANGLES_RAD[1], 1, leg);
        uint16_t knee_neutral = angle_to_servo_position(NEUTRAL_ANGLES_RAD[2] + NEUTRAL_ANGLES_RAD[1], 2, leg);  // knee as absolute
        RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), 
                    "  %s (leg_index=%zu): abd=%d, hip=%d, knee_abs=%d", 
                    leg_names[leg], leg, abd_neutral, hip_neutral, knee_neutral);
      }
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
  if (!esp32_interface_ || !esp32_interface_->is_connected())
  {
    // Don't spam errors - just skip this cycle
    return;
  }

  // Convert joint targets to calibrated servo positions.
  // Hardware servo order matches legacy MangDang PWMParams.servo_ids:
  // RF: (abd, hip, knee_abs) -> indices 0,1,2
  // LF: (abd, hip, knee_abs) -> indices 3,4,5
  // RB: (abd, hip, knee_abs) -> indices 6,7,8
  // LB: (abd, hip, knee_abs) -> indices 9,10,11
  std::array<uint16_t, ESP32Interface::NUM_SERVOS> servo_positions;
  servo_positions.fill(static_cast<uint16_t>(NEUTRAL_POSITION));

  // Extract commands from hw_position_commands_[]
  // joint_names_[0..2] are LF, [3..5] are RF, [6..8] are LB, [9..11] are RB
  const double lf_abd = hw_position_commands_[0];
  const double lf_hip = hw_position_commands_[1];
  const double lf_knee = hw_position_commands_[2];

  const double rf_abd = hw_position_commands_[3];
  const double rf_hip = hw_position_commands_[4];
  const double rf_knee = hw_position_commands_[5];

  const double lb_abd = hw_position_commands_[6];
  const double lb_hip = hw_position_commands_[7];
  const double lb_knee = hw_position_commands_[8];

  const double rb_abd = hw_position_commands_[9];
  const double rb_hip = hw_position_commands_[10];
  const double rb_knee = hw_position_commands_[11];

  // Legacy expects axis2 as absolute: hip + knee.
  const double rf_knee_abs = rf_hip + rf_knee;
  const double lf_knee_abs = lf_hip + lf_knee;
  const double rb_knee_abs = rb_hip + rb_knee;
  const double lb_knee_abs = lb_hip + lb_knee;

  // leg_index mapping: 0 RF, 1 LF, 2 RB, 3 LB
  servo_positions[0] = angle_to_servo_position(rf_abd, 0, 0);
  servo_positions[1] = angle_to_servo_position(rf_hip, 1, 0);
  servo_positions[2] = angle_to_servo_position(rf_knee_abs, 2, 0);

  servo_positions[3] = angle_to_servo_position(lf_abd, 0, 1);
  servo_positions[4] = angle_to_servo_position(lf_hip, 1, 1);
  servo_positions[5] = angle_to_servo_position(lf_knee_abs, 2, 1);

  servo_positions[6] = angle_to_servo_position(rb_abd, 0, 2);
  servo_positions[7] = angle_to_servo_position(rb_hip, 1, 2);
  servo_positions[8] = angle_to_servo_position(rb_knee_abs, 2, 2);

  servo_positions[9] = angle_to_servo_position(lb_abd, 0, 3);
  servo_positions[10] = angle_to_servo_position(lb_hip, 1, 3);
  servo_positions[11] = angle_to_servo_position(lb_knee_abs, 2, 3);

  // Debug: log servo positions occasionally
  static int debug_counter = 0;
  if (++debug_counter % 100 == 0)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("MiniPupperHardware"),
      "Servo positions: [%d,%d,%d, %d,%d,%d, %d,%d,%d, %d,%d,%d]",
      servo_positions[0], servo_positions[1], servo_positions[2],
      servo_positions[3], servo_positions[4], servo_positions[5],
      servo_positions[6], servo_positions[7], servo_positions[8],
      servo_positions[9], servo_positions[10], servo_positions[11]);
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

  // Debug: log raw servo positions occasionally
  static int read_debug_counter = 0;
  if (++read_debug_counter % 100 == 0)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("MiniPupperHardware"),
      "Read servo positions: [%d,%d,%d, %d,%d,%d, %d,%d,%d, %d,%d,%d]",
      servo_positions[0], servo_positions[1], servo_positions[2],
      servo_positions[3], servo_positions[4], servo_positions[5],
      servo_positions[6], servo_positions[7], servo_positions[8],
      servo_positions[9], servo_positions[10], servo_positions[11]);
  }

  // Convert servo raw values to radians

  // Decode hardware servo order back into URDF joint order.
  const double rf_abd = servo_position_to_angle(servo_positions[0], 0, 0);
  const double rf_hip = servo_position_to_angle(servo_positions[1], 1, 0);
  const double rf_knee_abs = servo_position_to_angle(servo_positions[2], 2, 0);
  const double rf_knee = rf_knee_abs - rf_hip;

  const double lf_abd = servo_position_to_angle(servo_positions[3], 0, 1);
  const double lf_hip = servo_position_to_angle(servo_positions[4], 1, 1);
  const double lf_knee_abs = servo_position_to_angle(servo_positions[5], 2, 1);
  const double lf_knee = lf_knee_abs - lf_hip;

  const double rb_abd = servo_position_to_angle(servo_positions[6], 0, 2);
  const double rb_hip = servo_position_to_angle(servo_positions[7], 1, 2);
  const double rb_knee_abs = servo_position_to_angle(servo_positions[8], 2, 2);
  const double rb_knee = rb_knee_abs - rb_hip;

  const double lb_abd = servo_position_to_angle(servo_positions[9], 0, 3);
  const double lb_hip = servo_position_to_angle(servo_positions[10], 1, 3);
  const double lb_knee_abs = servo_position_to_angle(servo_positions[11], 2, 3);
  const double lb_knee = lb_knee_abs - lb_hip;
  
  // Debug: log decoded joint angles occasionally
  if (read_debug_counter % 100 == 0)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("MiniPupperHardware"),
      "Decoded angles: LF[%.3f,%.3f,%.3f] RF[%.3f,%.3f,%.3f] LB[%.3f,%.3f,%.3f] RB[%.3f,%.3f,%.3f]",
      lf_abd, lf_hip, lf_knee, rf_abd, rf_hip, rf_knee, 
      lb_abd, lb_hip, lb_knee, rb_abd, rb_hip, rb_knee);
  }

  // Write state back to hw_positions_[] in joint_names_ order
  hw_positions_[0] = lf_abd;
  hw_positions_[1] = lf_hip;
  hw_positions_[2] = lf_knee;

  hw_positions_[3] = rf_abd;
  hw_positions_[4] = rf_hip;
  hw_positions_[5] = rf_knee;

  hw_positions_[6] = lb_abd;
  hw_positions_[7] = lb_hip;
  hw_positions_[8] = lb_knee;

  hw_positions_[9] = rb_abd;
  hw_positions_[10] = rb_hip;
  hw_positions_[11] = rb_knee;
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

  // Invert: servo_position = neutral - micros_per_rad * ((angle - neutral_angle) * multiplier)
  const double delta = (NEUTRAL_POSITION - static_cast<double>(servo_position)) / MICROS_PER_RAD;
  return neutral_angle + (delta / static_cast<double>(multiplier));
}

void MiniPupperHardware::build_joint_mapping()
{
  // Nothing to do - we use joint_names_ order directly
  // StateInterfaces bind joint names to hw_positions_ array indices 1:1
  RCLCPP_INFO(rclcpp::get_logger("MiniPupperHardware"), "Using joint order as-is from ros2_control");
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
