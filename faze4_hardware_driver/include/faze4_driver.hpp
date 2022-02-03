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

#ifndef FAZE4_DRIVER_HPP_
#define FAZE4_DRIVER_HPP_

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "timeout_serial.hpp"
#include "visibility_control.h"

namespace faze4_hardware_driver
{
class Faze4HardwareDriver
{
public:
  FAZE4_HARDWARE_DRIVER_PUBLIC
  bool init(const std::string & device, int baudrate);

  FAZE4_HARDWARE_DRIVER_PUBLIC
  void set_angles(std::vector<double> angles);

  FAZE4_HARDWARE_DRIVER_PUBLIC
  void get_angles(std::vector<double> & angles);

  FAZE4_HARDWARE_DRIVER_PUBLIC
  void home(void);

private:
  std::shared_ptr<TimeoutSerial> serial_;
};

}  // namespace faze4_hardware_driver

#endif  // FAZE4_DRIVER_HPP_
