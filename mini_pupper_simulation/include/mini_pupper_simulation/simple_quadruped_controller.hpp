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

namespace mini_pupper_simulation
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

  std::vector<std::string> joint_names_;
  std::vector<double> default_positions_;
  std::vector<double> commanded_positions_;

  rclcpp::Subscription<CommandMsg>::SharedPtr command_subscription_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CommandMsg>> command_buffer_;
  bool has_external_command_{false};
};
}  // namespace mini_pupper_simulation
