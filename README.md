# faze4_ros

[![Build Status](https://github.com/HX2003/faze4_ros/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=main)](https://github.com/HX2003/faze4_ros/actions/workflows/rolling-semi-binary-build.yml?branch=main)
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository allows one to control [FAZE4 robotic arm](https://github.com/PCrnjak/Faze4-Robotic-arm) with MoveIt Motion Planning Framework for ROS 2.

# ESP32 Embedded Controller

Compile and flash this Arduino [sketch](https://github.com/HX2003/faze4_ros/blob/main/faze4_embedded_controller/ESP32Faze4Controller/ESP32Faze4Controller.ino) onto an ESP32.

# Building Source
```
cd src
git clone https://github.com/ros-controls/ros2_control
git clone https://github.com/ros-controls/ros2_controllers
git clone https://github.com/HX2003/faze4_ros
cd ..
```

* Install dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-rolling-moveit-simple-controller-manager
```

* Source ros *
```
source /opt/ros/rolling/setup.bash
```

* Build everything with:
```
colcon build --symlink-install
```

* Source colcon workspace:
 ```
source ./install/setup.bash
 ```

# Launch options
With mcu connected via serial
 ```
ros2 launch faze4_moveit_config demo.launch.py
 ```
 With mcu connected via serial (serial configs)
 ```
ros2 launch faze4_moveit_config demo.launch.py serial_device:='/dev/ttyUSB0' serial_baudrate:='115200'
 ```
Without any hardware connected via serial
 ```
ros2 launch faze4_moveit_config demo.launch.py use_fake_hardware:='true'
```
