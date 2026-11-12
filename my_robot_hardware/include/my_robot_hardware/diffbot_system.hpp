#pragma once

#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot_hardware
{

class DiffBotSystem : public hardware_interface::SystemInterface
{
public:
  // 1. Read the <ros2_control> block of the URDF (joints, interfaces, params)
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // 2. Lifecycle: open / close the connection, enable / disable motors
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // 3. Interfaces that controllers can claim
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 4. Called every control cycle by the controller_manager
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> cmd_velocity_;   // written by controllers
  std::vector<double> state_position_; // read by controllers
  std::vector<double> state_velocity_;
};

}  // namespace my_robot_hardware
