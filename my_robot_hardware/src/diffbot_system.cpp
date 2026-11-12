#include "my_robot_hardware/diffbot_system.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn DiffBotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // info_.joints holds every <joint> of the <ros2_control> block
  const auto n = info_.joints.size();
  cmd_velocity_.assign(n, 0.0);
  state_position_.assign(n, 0.0);
  state_velocity_.assign(n, 0.0);
  // Example of a custom <param name="port">/dev/ttyUSB0</param> in the URDF:
  // const std::string port = info_.hardware_parameters.at("port");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystem::on_configure(const rclcpp_lifecycle::State &)
{
  // Open the serial port / CAN bus / socket here and check the board answers
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystem"), "Connected to the motor board");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Close the connection opened in on_configure()
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Enable the motor drivers. Start from zero velocity.
  std::fill(cmd_velocity_.begin(), cmd_velocity_.end(), 0.0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop and disable the motors (safe state)
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // name = "<joint>/position": the same strings you see in list_hardware_interfaces
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commands;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    commands.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocity_[i]);
  }
  return commands;
}

return_type DiffBotSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // REAL ROBOT: read the encoders here. This demo pretends the wheels follow
  // the command perfectly and integrates the position.
  for (size_t i = 0; i < state_velocity_.size(); ++i) {
    state_velocity_[i] = cmd_velocity_[i];
    state_position_[i] += state_velocity_[i] * period.seconds();
  }
  return return_type::OK;
}

return_type DiffBotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // REAL ROBOT: send cmd_velocity_ (rad/s) to the motor board here. Keep it fast:
  // no blocking calls, no memory allocation.
  return return_type::OK;
}

}  // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::DiffBotSystem, hardware_interface::SystemInterface)
