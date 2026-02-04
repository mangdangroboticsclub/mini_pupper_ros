#include "mini_pupper_controllers/simple_quadruped_controller.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
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
  if (!node)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SimpleQuadrupedController"), "Unable to access node interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!node->has_parameter("joints"))
  {
    node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  }
  if (!node->has_parameter("default_positions"))
  {
    node->declare_parameter<std::vector<double>>("default_positions", std::vector<double>{});
  }
  command_buffer_.writeFromNonRT(nullptr);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SimpleQuadrupedController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());

  for (const auto & joint : joint_names_)
  {
    config.names.emplace_back(joint + "/position");
  }

  return config;
}

controller_interface::InterfaceConfiguration SimpleQuadrupedController::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(joint_names_.size());

  for (const auto & joint : joint_names_)
  {
    config.names.emplace_back(joint + "/position");
  }

  return config;
}

controller_interface::CallbackReturn SimpleQuadrupedController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  if (!node)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SimpleQuadrupedController"), "Unable to access node interface");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_ = node->get_parameter("joints").as_string_array();
  default_positions_ = node->get_parameter("default_positions").as_double_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!default_positions_.empty() && default_positions_.size() != joint_names_.size())
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Parameter 'default_positions' size (%zu) does not match joints size (%zu); padding with zeros.",
      default_positions_.size(), joint_names_.size());
  }

  assign_default_if_needed();
  commanded_positions_ = default_positions_;
  has_external_command_ = false;

  // Log the default positions being used
  RCLCPP_INFO(node->get_logger(), "Controller initialized with default_positions:");
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    RCLCPP_INFO(node->get_logger(), "  %s: %.4f rad (%.2f deg)", 
                joint_names_[i].c_str(), default_positions_[i], 
                default_positions_[i] * 180.0 / M_PI);
  }

  command_buffer_.writeFromNonRT(nullptr);

  command_subscription_ = node->create_subscription<CommandMsg>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CommandMsg::SharedPtr msg) { command_buffer_.writeFromNonRT(msg); });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleQuadrupedController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  assign_default_if_needed();
  commanded_positions_ = default_positions_;
  has_external_command_ = false;
  command_buffer_.writeFromNonRT(nullptr);

  if (command_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Number of command interfaces (%zu) does not match joints (%zu)",
      command_interfaces_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (size_t index = 0; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value(commanded_positions_[index]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleQuadrupedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  commanded_positions_.clear();
  has_external_command_ = false;
  command_buffer_.writeFromNonRT(nullptr);
  command_subscription_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SimpleQuadrupedController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto command_ptr = command_buffer_.readFromRT();
  if (command_ptr && *command_ptr)
  {
    const auto & msg = *(*command_ptr);
    if (msg.data.size() == joint_names_.size())
    {
      commanded_positions_.assign(msg.data.begin(), msg.data.end());
      has_external_command_ = true;
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "Command size (%zu) does not match joints (%zu)",
        msg.data.size(), joint_names_.size());
    }
  }

  if (!has_external_command_)
  {
    commanded_positions_ = default_positions_;
  }

  const auto & target_positions = commanded_positions_;

  // Debug: Log what controller is commanding
  static int controller_log_counter = 0;
  if (++controller_log_counter % 100 == 0)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Controller commanding positions (rad): [%.3f,%.3f,%.3f, %.3f,%.3f,%.3f, %.3f,%.3f,%.3f, %.3f,%.3f,%.3f]",
      target_positions[0], target_positions[1], target_positions[2],
      target_positions[3], target_positions[4], target_positions[5],
      target_positions[6], target_positions[7], target_positions[8],
      target_positions[9], target_positions[10], target_positions[11]);
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Using %s commands", has_external_command_ ? "EXTERNAL" : "DEFAULT");
  }

  if (command_interfaces_.size() != target_positions.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Mismatch between command interfaces (%zu) and target positions (%zu)",
      command_interfaces_.size(), target_positions.size());
    return controller_interface::return_type::ERROR;
  }

  for (size_t index = 0; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value(target_positions[index]);
  }

  return controller_interface::return_type::OK;
}

void SimpleQuadrupedController::assign_default_if_needed()
{
  auto node = get_node();
  if (default_positions_.size() != joint_names_.size())
  {
    default_positions_.assign(joint_names_.size(), 0.0);
  }

  if (commanded_positions_.size() != joint_names_.size())
  {
    commanded_positions_.assign(joint_names_.size(), 0.0);
  }
}

}  // namespace mini_pupper_controllers

PLUGINLIB_EXPORT_CLASS(
  mini_pupper_controllers::SimpleQuadrupedController, controller_interface::ControllerInterface)
