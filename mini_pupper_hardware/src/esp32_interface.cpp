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

#include "mini_pupper_hardware/esp32_interface.hpp"

#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace mini_pupper_hardware
{
ESP32Interface::ESP32Interface(const std::string & socket_path)
: socket_path_(socket_path)
{
  connect();
}

ESP32Interface::~ESP32Interface()
{
  close();
}

ESP32Interface::ESP32Interface(ESP32Interface && other) noexcept
: socket_fd_(other.socket_fd_), socket_path_(other.socket_path_)
{
  other.socket_fd_ = -1;
}

ESP32Interface & ESP32Interface::operator=(ESP32Interface && other) noexcept
{
  if (this != &other)
  {
    close();
    socket_fd_ = other.socket_fd_;
    socket_path_ = other.socket_path_;
    other.socket_fd_ = -1;
  }
  return *this;
}

bool ESP32Interface::connect()
{
  close();

  // Create Unix domain socket
  socket_fd_ = socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ESP32Interface"),
      "Failed to create socket: %s", strerror(errno));
    return false;
  }

  // Set socket timeout (10ms for 100Hz control loop)
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 10000;  // 10ms

  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ESP32Interface"),
      "Failed to set recv timeout: %s", strerror(errno));
  }

  if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ESP32Interface"),
      "Failed to set send timeout: %s", strerror(errno));
  }

  // Connect to the socket
  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (::connect(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ESP32Interface"),
      "Failed to connect to %s: %s", socket_path_.c_str(), strerror(errno));
    close();
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ESP32Interface"),
    "Connected to ESP32 proxy at %s", socket_path_.c_str());
  return true;
}

void ESP32Interface::close()
{
  if (socket_fd_ >= 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool ESP32Interface::is_connected() const
{
  return socket_fd_ >= 0;
}

std::vector<uint8_t> ESP32Interface::send_and_receive(
  const std::vector<uint8_t> & send_data, size_t expected_response_size)
{
  std::vector<uint8_t> response;

  if (!is_connected())
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32Interface"), "Socket not connected");
    return response;
  }

  // Send data
  if (send(socket_fd_, send_data.data(), send_data.size(), 0) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ESP32Interface"), "Send failed: %s", strerror(errno));
    handle_socket_error();
    return response;
  }

  // Receive response
  std::vector<uint8_t> buffer(expected_response_size);
  ssize_t bytes_received = recv(socket_fd_, buffer.data(), expected_response_size, 0);

  if (bytes_received < 0)
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("ESP32Interface"),
        "Recv timeout - no response from ESP32");
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ESP32Interface"), "Recv failed: %s", strerror(errno));
    }
    handle_socket_error();
    return response;
  }

  response.assign(buffer.begin(), buffer.begin() + bytes_received);
  return response;
}

void ESP32Interface::handle_socket_error()
{
  if (errno == EPIPE || errno == ENOTCONN || errno == EBADF)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ESP32Interface"),
      "Socket error, attempting reconnection");
    close();
    connect();
  }
}

bool ESP32Interface::servos_set_position_torque(
  const std::array<uint16_t, NUM_SERVOS> & positions,
  const std::array<uint16_t, NUM_SERVOS> & torque)
{
  // Protocol: BB12B12H  (matches installed esp32-proxy on Mini Pupper 2)
  // B:  packet size (38 = 2 + 12 + 24)
  // B:  command type (1 = INST_SETPOS)
  // 12B: torque values as uint8 (0=disabled, 1=enabled)
  // 12H: position values as uint16 little-endian

  std::vector<uint8_t> send_data;
  send_data.push_back(38);  // total packet size
  send_data.push_back(1);   // command type

  // Add torque values as single bytes (uint8)
  for (const auto & t : torque)
  {
    send_data.push_back(static_cast<uint8_t>(t & 0xFF));
  }

  // Add position values (uint16 little-endian)
  for (const auto & p : positions)
  {
    send_data.push_back(p & 0xFF);
    send_data.push_back((p >> 8) & 0xFF);
  }

  RCLCPP_DEBUG(rclcpp::get_logger("ESP32Interface"), "Sending position command");
  auto response = send_and_receive(send_data, 2);

  if (response.empty())
  {
    static int no_response_count = 0;
    if (++no_response_count % 10 == 0)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("ESP32Interface"),
        "No response from ESP32 proxy (count: %d)", no_response_count);
    }
    return false;
  }

  if (response.size() != 2 || response[0] != 2 || response[1] != 1)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ESP32Interface"),
      "Invalid acknowledgment: size=%zu, [0]=%d, [1]=%d",
      response.size(), response.size() > 0 ? response[0] : -1,
      response.size() > 1 ? response[1] : -1);
    return false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("ESP32Interface"), "Position command ACK received");
  return true;
}

bool ESP32Interface::servos_set_position(
  const std::array<uint16_t, NUM_SERVOS> & positions)
{
  std::array<uint16_t, NUM_SERVOS> torque;
  torque.fill(1);  // Binary enable (BB12B12H protocol: torque is uint8, 1=enabled)
  return servos_set_position_torque(positions, torque);
}

std::vector<uint16_t> ESP32Interface::servos_get_position()
{
  // Protocol: BB
  // B: packet size (2)
  // B: command type (2)
  std::vector<uint8_t> send_data = {2, 2};
  auto response = send_and_receive(send_data, 26);

  std::vector<uint16_t> result;

  if (response.size() != 26 || response[0] != 26 || response[1] != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32Interface"), "Invalid get_position response");
    return result;
  }

  // Parse 12 uint16_t values (little-endian)
  for (size_t i = 0; i < NUM_SERVOS; ++i)
  {
    uint16_t value = response[2 + i * 2] | (response[2 + i * 2 + 1] << 8);
    result.push_back(value);
  }

  return result;
}

std::vector<int16_t> ESP32Interface::servos_get_torque()
{
  // Protocol: BB
  // B: packet size (2)
  // B: command type (6)
  std::vector<uint8_t> send_data = {2, 6};
  auto response = send_and_receive(send_data, 26);

  std::vector<int16_t> result;

  if (response.size() != 26 || response[0] != 26 || response[1] != 6)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32Interface"), "Invalid get_torque response");
    return result;
  }

  // Parse 12 int16_t values (little-endian)
  for (size_t i = 0; i < NUM_SERVOS; ++i)
  {
    int16_t value = response[2 + i * 2] | (response[2 + i * 2 + 1] << 8);
    result.push_back(value);
  }

  return result;
}

std::vector<int16_t> ESP32Interface::servos_get_load()
{
  // Protocol: BB
  // B: packet size (2)
  // B: command type (3)
  std::vector<uint8_t> send_data = {2, 3};
  auto response = send_and_receive(send_data, 26);

  std::vector<int16_t> result;

  if (response.size() != 26 || response[0] != 26 || response[1] != 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32Interface"), "Invalid get_load response");
    return result;
  }

  // Parse 12 int16_t values (little-endian)
  for (size_t i = 0; i < NUM_SERVOS; ++i)
  {
    int16_t value = response[2 + i * 2] | (response[2 + i * 2 + 1] << 8);
    result.push_back(value);
  }

  return result;
}

std::vector<float> ESP32Interface::imu_get_data()
{
  // Protocol: BB
  // B: packet size (2)
  // B: command type (4)
  std::vector<uint8_t> send_data = {2, 4};
  auto response = send_and_receive(send_data, 26);

  std::vector<float> result;

  if (response.size() != 26 || response[0] != 26 || response[1] != 4)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32Interface"), "Invalid imu_get_data response");
    return result;
  }

  // Parse 6 float values
  for (size_t i = 0; i < 6; ++i)
  {
    float value;
    memcpy(&value, &response[2 + i * 4], sizeof(float));
    result.push_back(value);
  }

  return result;
}

std::vector<float> ESP32Interface::get_power_status()
{
  // Protocol: BB
  // B: packet size (2)
  // B: command type (5)
  std::vector<uint8_t> send_data = {2, 5};
  auto response = send_and_receive(send_data, 10);

  std::vector<float> result;

  if (response.size() != 10 || response[0] != 10 || response[1] != 5)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ESP32Interface"), "Invalid get_power_status response");
    return result;
  }

  // Parse 2 float values
  for (size_t i = 0; i < 2; ++i)
  {
    float value;
    memcpy(&value, &response[2 + i * 4], sizeof(float));
    result.push_back(value);
  }

  return result;
}

bool ESP32Interface::save_calibration()
{
  // Save calibration by setting torque to 99 (invalid value), then reset
  std::array<uint16_t, NUM_SERVOS> calibration_torque;
  calibration_torque.fill(99);
  std::array<uint16_t, NUM_SERVOS> dummy_positions;
  dummy_positions.fill(1);

  if (!servos_set_position_torque(dummy_positions, calibration_torque))
  {
    return false;
  }

  // Allow time for ESP32 to process
  usleep(500000);  // 500ms

  // Reset to normal torque and neutral position
  std::array<uint16_t, NUM_SERVOS> normal_torque;
  normal_torque.fill(1);
  std::array<uint16_t, NUM_SERVOS> neutral_positions;
  neutral_positions.fill(512);

  return servos_set_position_torque(neutral_positions, normal_torque);
}

}  // namespace mini_pupper_hardware
