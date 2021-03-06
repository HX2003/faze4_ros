# faze4_ros

[![Build Status](https://github.com/HX2003/faze4_ros/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=main)](https://github.com/HX2003/faze4_ros/actions/workflows/rolling-semi-binary-build.yml?branch=main)
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This repository allows one to control [FAZE4 robotic arm](https://github.com/PCrnjak/Faze4-Robotic-arm) with MoveIt Motion Planning Framework for ROS 2.

# ESP32 Embedded Controller

Compile and flash this Arduino [sketch](https://github.com/HX2003/faze4_ros/blob/main/faze4_embedded_controller/ESP32Faze4Controller/ESP32Faze4Controller.ino) onto an ESP32.

# Building Source
```
cd src
git clone https://github.com/HX2003/faze4_ros
vcs import < faze4_ros/faze4_ros.repos
cd ..
```

* Install dependencies:
```
rosdep install --from-paths src --ignore-src -r -y
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

# Interactive Marker
Drag the marker around and click plan & execute in the MotionPlanning panel.

# Faze4Gui
Click plan & execute in the Faze4Gui panel to run one of the following demos
- Box Constraints Demo
- Plane Constraints Demo
- Line Constraints Demo
- Orientation Constraints Demo
