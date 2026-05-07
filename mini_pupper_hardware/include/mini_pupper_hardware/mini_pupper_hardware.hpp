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

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "mini_pupper_hardware/esp32_interface.hpp"

namespace mini_pupper_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MiniPupperHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // Mini Pupper has 12 servos: 4 legs * 3 joints each (abduction, hip, knee)
  // Hardware expects the 3rd joint per leg as an absolute angle (hip + knee),
  // matching the legacy MangDang Python HardwareInterface.
  static constexpr size_t NUM_JOINTS = 12;

  // Joint names extracted from URDF (in ros2_control order)
  std::vector<std::string> joint_names_;

  // Maps hardware servo order (RF, LF, RB, LB x abd, hip, knee) to the
  // corresponding joint index in joint_names_.
  std::array<size_t, NUM_JOINTS> hardware_joint_to_urdf_index_{};

  // Legacy servo calibration model (mirrors MangDang Python Config/HardwareInterface)
  // - neutral position at 512
  // - per-axis neutral angles (0, +45deg, -45deg)
  // - per-leg direction multipliers
  static constexpr double NEUTRAL_POSITION = 512.0;
  static constexpr double MICROS_PER_RAD = (760.0 - 210.0) / M_PI;
  static constexpr std::array<double, 3> NEUTRAL_ANGLES_RAD = {0.0, M_PI_4, -M_PI_4};

  // Multipliers indexed by [axis][leg], where leg order is:
  // 0: front-right (RF), 1: front-left (LF), 2: back-right (RB), 3: back-left (LB)
  // NOTE: Front legs (RF, LF) use +1 for abduction; back legs (RB, LB) use -1
  static constexpr std::array<std::array<int, 4>, 3> SERVO_MULTIPLIERS = {
    std::array<int, 4>{1, 1, -1, -1},  // axis 0 (abduction)
    std::array<int, 4>{-1, 1, -1, 1},    // axis 1 (hip)
    std::array<int, 4>{-1, 1, -1, 1},    // axis 2 (knee)
  };

  // Joint state: [position, velocity, effort] for each joint
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Joint position command for each joint
  std::vector<double> hw_position_commands_;

  // Previous position for velocity calculation
  std::vector<double> hw_positions_prev_;

  // Per-instance divider for throttling blocking hardware reads.
  int read_counter_ = 0;

  // Configuration parameters
  std::string hardware_interface_type_;  // "mock", "esp32_proxy", etc.
  bool use_mock_hardware_ = false;

  // ESP32 interface for real hardware
  std::unique_ptr<ESP32Interface> esp32_interface_;

  // Clock for throttled logging
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Helper methods
  void initialize_state_storage();
  bool build_joint_mapping();
  void update_velocities(const rclcpp::Duration & period);
  void send_commands_to_hardware();
  void read_state_from_hardware();

  uint16_t angle_to_servo_position(double angle_rad, size_t axis_index, size_t leg_index);
  double servo_position_to_angle(uint16_t servo_position, size_t axis_index, size_t leg_index);
};

}  // namespace mini_pupper_hardware

