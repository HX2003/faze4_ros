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

#ifndef FAZE4_GUI__FAZE4_MOTION_HPP_
#define FAZE4_GUI__FAZE4_MOTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/constraints.hpp>

namespace faze4_gui
{
class Faze4Motion
{
public:
  Faze4Motion(const rclcpp::Node::SharedPtr & node);
  void debug();
  void moveToTarget(tf2::Vector3 pos, tf2::Quaternion q);
  void boxConstraintsDemo();
  void planeConstraintsDemo();
  void lineConstraintsDemo();
  void orientationConstraintsDemo();

private:
  bool planPathAndExecute();
  geometry_msgs::msg::PoseStamped createPoseGoal(double dx, double dy, double dz);
  moveit_msgs::msg::PositionConstraint createBoxConstraint();
  moveit_msgs::msg::PositionConstraint createPlaneConstraint();
  moveit_msgs::msg::PositionConstraint createLineConstraint();
  moveit_msgs::msg::PositionConstraint createOrientationConstraint();

  rclcpp::Node::SharedPtr node_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::string ref_link_, ee_link_;
};

}  // namespace faze4_gui

#endif  // FAZE4_GUI__FAZE4_MOTION_HPP_
