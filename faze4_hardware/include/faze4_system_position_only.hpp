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

#ifndef FAZE4_HARDWARE__FAZE4_SYSTEM_POSITION_ONLY_HPP_
#define FAZE4_HARDWARE__FAZE4_SYSTEM_POSITION_ONLY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "faze4_driver.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace faze4_hardware
{
class Faze4SystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Faze4SystemPositionOnlyHardware);

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  bool use_fake_hardware_;
  std::string serial_device_;
  int serial_baudrate_;

  faze4_hardware_driver::Faze4HardwareDriver driver;
};

}  // namespace faze4_hardware

#endif  // FAZE4_HARDWARE__FAZE4_SYSTEM_POSITION_ONLY_HPP_
