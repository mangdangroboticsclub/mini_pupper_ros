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
#include <cstdint>
#include <string>
#include <vector>

namespace mini_pupper_hardware
{
/**
 * C++ interface to ESP32 proxy socket server.
 * Mirrors the Python ESP32Interface for servo control.
 *
 * Communicates with esp32-proxy daemon via Unix socket at /tmp/esp32-proxy.socket
 */
class ESP32Interface
{
public:
  static constexpr int NUM_SERVOS = 12;
  static constexpr const char * SOCKET_PATH = "/tmp/esp32-proxy.socket";

  explicit ESP32Interface(const std::string & socket_path = SOCKET_PATH);
  ~ESP32Interface();

  // Disable copy operations
  ESP32Interface(const ESP32Interface &) = delete;
  ESP32Interface & operator=(const ESP32Interface &) = delete;

  // Move operations
  ESP32Interface(ESP32Interface && other) noexcept;
  ESP32Interface & operator=(ESP32Interface && other) noexcept;

  /**
   * Connect to ESP32 proxy socket.
   * Called automatically in constructor, can be called again to reconnect.
   */
  bool connect();

  /**
   * Close the socket connection.
   */
  void close();

  /**
   * Check if socket is connected.
   */
  bool is_connected() const;

  /**
   * Set servo positions and torques.
   *
   * @param positions Array of 12 servo positions (0-1023 raw values)
   * @param torque Array of 12 torque values (1-1023, or 99 for calibration)
   * @return true if successful, false otherwise
   */
  bool servos_set_position_torque(
    const std::array<uint16_t, NUM_SERVOS> & positions,
    const std::array<uint16_t, NUM_SERVOS> & torque);

  /**
   * Set servo positions with default torque (all 1).
   *
   * @param positions Array of 12 servo positions (0-1023 raw values)
   * @return true if successful, false otherwise
   */
  bool servos_set_position(const std::array<uint16_t, NUM_SERVOS> & positions);

  /**
   * Get current servo positions.
   *
   * @return Vector of 12 servo positions, empty if error
   */
  std::vector<uint16_t> servos_get_position();

  /**
   * Get current servo torques/loads.
   *
   * @return Vector of 12 torque values, empty if error
   */
  std::vector<int16_t> servos_get_torque();

  /**
   * Get servo load (current draw).
   *
   * @return Vector of 12 load values, empty if error
   */
  std::vector<int16_t> servos_get_load();

  /**
   * Get IMU data (accelerometer and gyroscope).
   *
   * @return Array of 6 float values [ax, ay, az, gx, gy, gz], empty if error
   */
  std::vector<float> imu_get_data();

  /**
   * Get power status (voltage and current).
   *
   * @return Array of 2 float values [voltage, current], empty if error
   */
  std::vector<float> get_power_status();

  /**
   * Save calibration to ESP32 flash.
   *
   * @return true if successful, false otherwise
   */
  bool save_calibration();

private:
  int socket_fd_ = -1;
  std::string socket_path_;

  /**
   * Handle socket errors and attempt reconnection if needed.
   */
  void handle_socket_error();

  /**
   * Send data and receive response.
   *
   * @param send_data Data to send
   * @param expected_response_size Expected response size
   * @return Response data, empty if error
   */
  std::vector<uint8_t> send_and_receive(
    const std::vector<uint8_t> & send_data, size_t expected_response_size);
};

}  // namespace mini_pupper_hardware

