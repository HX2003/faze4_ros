// Copyright 2020 ros2_control Development Team
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

#include "faze4_system_position_only.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "faze4_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace faze4_hardware
{
hardware_interface::CallbackReturn Faze4SystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string use_fake_hardware_str = info_.hardware_parameters["use_fake_hardware"];
  std::transform(
    use_fake_hardware_str.begin(), use_fake_hardware_str.end(), use_fake_hardware_str.begin(),
    ::tolower);
  std::istringstream is(use_fake_hardware_str);
  is >> std::boolalpha >> use_fake_hardware_;

  serial_device_ = info_.hardware_parameters["serial_device"];
  serial_baudrate_ = std::stod(info_.hardware_parameters["serial_baudrate"]);

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Faze4SystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Faze4SystemPositionOnlyHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Faze4SystemPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Faze4SystemPositionOnlyHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Faze4SystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Faze4SystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Faze4SystemPositionOnlyHardware"), "Configuring");

  if (use_fake_hardware_)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("Faze4SystemPositionOnlyHardware"),
      "Using fake hardware, not initialising serial");
  }
  else
  {
    RCLCPP_INFO(
      rclcpp::get_logger("Faze4SystemPositionOnlyHardware"),
      "Initialising serial with serial_device=%s\nserial_baudrate=%d", serial_device_.c_str(),
      serial_baudrate_);

    if (!driver.init(serial_device_, serial_baudrate_))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("Faze4SystemPositionOnlyHardware"), "Failed to initialise serial");
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Faze4SystemPositionOnlyHardware"), "System Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Faze4SystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Faze4SystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Faze4SystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Faze4SystemPositionOnlyHardware"), "Starting ...please wait...");

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Faze4SystemPositionOnlyHardware"), "System Successfully started!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Faze4SystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Faze4SystemPositionOnlyHardware"), "Stopping ...please wait...");

  RCLCPP_INFO(
    rclcpp::get_logger("Faze4SystemPositionOnlyHardware"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Faze4SystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_fake_hardware_)
  {
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      // Simulate receiving commands from the hardware
      hw_states_[i] = hw_commands_[i];
      RCLCPP_INFO(
        rclcpp::get_logger("Faze4SystemPositionOnlyHardware"),
        "Simulated: Got state %.5f for joint %d!", hw_states_[i], i + 1);
    }
  }
  else
  {
    driver.get_angles(hw_states_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Faze4SystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_fake_hardware_)
  {
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      // Simulate sending commands to the hardware
      RCLCPP_INFO(
        rclcpp::get_logger("Faze4SystemPositionOnlyHardware"),
        "Simulated: Got command %.5f for joint %d!", hw_commands_[i], i + 1);
    }
  }
  else
  {
    driver.set_angles(hw_commands_);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace faze4_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  faze4_hardware::Faze4SystemPositionOnlyHardware, hardware_interface::SystemInterface)
