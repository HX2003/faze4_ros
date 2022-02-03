// Copyright 2022 HX2003
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

#include "faze4_driver.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include "rclcpp/rclcpp.hpp"

namespace faze4_hardware_driver
{
bool Faze4HardwareDriver::init(const std::string & device, int baudrate)
{
  try
  {
    serial_ = std::make_shared<TimeoutSerial>(
      device, baudrate,
      boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
      boost::asio::serial_port_base::character_size(8),
      boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none),
      boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    serial_->setTimeout(boost::posix_time::seconds(1));
    serial_->writeString("INIT: \n");
  }
  catch (boost::system::system_error & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Faze4HardwareDriver"), "%s", e.what());
    return false;
  }

  return true;
}

void Faze4HardwareDriver::set_angles(std::vector<double> angles)
{
  boost::format fmt = boost::format("SET: %6.5f, %6.5f, %6.5f, %6.5f, %6.5f, %6.5f\n") % angles[0] %
                      angles[1] % angles[2] % angles[3] % angles[4] % angles[5];

  serial_->writeString(fmt.str());
}

void Faze4HardwareDriver::get_angles(std::vector<double> & angles)
{
  serial_->writeString("GET:\n");

  std::string inputString;
  try
  {
    inputString = serial_->readStringUntil("\n");
  }
  catch (timeout_exception & e)
  {
    RCLCPP_INFO(rclcpp::get_logger("Faze4HardwareDriver"), "Timeout!");
    return;
  }
  catch (boost::system::system_error & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Faze4HardwareDriver"), "%s", e.what());
    return;
  }

  std::sscanf(
    inputString.c_str(), "SEND: %lf, %lf, %lf, %lf, %lf, %lf", &angles[0], &angles[1], &angles[2],
    &angles[3], &angles[4], &angles[5]);
}

void Faze4HardwareDriver::home(void) { serial_->writeString("HOME:\n"); }
}  // namespace faze4_hardware_driver
