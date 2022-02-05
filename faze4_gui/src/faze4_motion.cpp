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

#include "include/faze4_motion.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/constraints.hpp>

static const std::string PLANNING_GROUP = "faze4_arm";
namespace faze4_gui
{
Faze4Motion::Faze4Motion(const rclcpp::Node::SharedPtr & node)
{
  move_group_ =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
  move_group_->setPlanningTime(10.0);
  move_group_->setNumPlanningAttempts(5);
  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);

  ref_link_ = move_group_->getPoseReferenceFrame();
  ee_link_ = move_group_->getEndEffectorLink();

  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
    node, "faze4_base_link", "visualization_marker_array", move_group_->getRobotModel());
}

bool Faze4Motion::planPathAndExecute()
{
  //Display start and target first since planning can take some time
  visual_tools_->publishSphere(
    move_group_->getCurrentPose().pose, rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
  visual_tools_->publishSphere(
    move_group_->getPoseTarget().pose, rviz_visual_tools::GREEN, rviz_visual_tools::XLARGE);
  visual_tools_->trigger();

  RCLCPP_INFO(rclcpp::get_logger("Faze4Motion"), "Planning...");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(rclcpp::get_logger("Faze4Motion"), "Planning %s", success ? "success" : "failed");

  if (!success) return false;

  moveit_msgs::msg::Constraints constraints = move_group_->getPathConstraints();
  const std::size_t num_pos_con = constraints.position_constraints.size();
  const std::size_t num_ori_con = constraints.orientation_constraints.size();

  rviz_visual_tools::Colors color =
    (num_pos_con > 0 || num_ori_con > 0) ? rviz_visual_tools::ORANGE : rviz_visual_tools::GREEN;

  visual_tools_->publishTrajectoryLine(
    plan.trajectory_, move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP), color);
  visual_tools_->trigger();

  move_group_->execute(plan);
  return true;
}

void Faze4Motion::debug()
{
  const Eigen::Isometry3d & tip_pose =
    move_group_->getCurrentState()->getGlobalLinkTransform("faze4_link7");

  visual_tools_->publishSphere(tip_pose, rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
  visual_tools_->trigger();
}

void Faze4Motion::moveToTarget(tf2::Vector3 pos, tf2::Quaternion q)
{
  visual_tools_->deleteAllMarkers();
  move_group_->clearPoseTargets();
  move_group_->clearPathConstraints();

  RCLCPP_INFO(rclcpp::get_logger("Faze4Motion"), "moveToTarget");

  move_group_->setStartStateToCurrentState();

  geometry_msgs::msg::Pose target_pose;
  geometry_msgs::msg::Point point;
  tf2::toMsg(pos, point);
  target_pose.orientation = tf2::toMsg(q);
  target_pose.position = point;
  move_group_->setPoseTarget(target_pose);

  planPathAndExecute();

  visual_tools_->deleteAllMarkers();
  move_group_->clearPoseTargets();
  move_group_->clearPathConstraints();
}

void Faze4Motion::boxConstraintsDemo()
{
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 2);
  moveToTarget(tf2::Vector3(-0.3, -0.5, 0.5), q);

  RCLCPP_INFO(rclcpp::get_logger("Faze4Motion"), "boxConstraintsDemo");

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools_->publishText(
    text_pose, "boxConstraintsDemo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

  const moveit::core::RobotStatePtr start_state = move_group_->getCurrentState();
  const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.3, -0.3);
  const moveit_msgs::msg::PositionConstraint pcm = createBoxConstraint();

  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.name = "box constraints";
  path_constraints.position_constraints.emplace_back(pcm);

  move_group_->setStartState(*start_state);
  move_group_->setPoseTarget(pose_goal);
  move_group_->setPathConstraints(path_constraints);

  planPathAndExecute();
}

void Faze4Motion::planeConstraintsDemo()
{
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 2);
  moveToTarget(tf2::Vector3(-0.3, -0.5, 0.5), q);

  RCLCPP_INFO(rclcpp::get_logger("Faze4Motion"), "planeConstraintsDemo");

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools_->publishText(
    text_pose, "planeConstraintsDemo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

  const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.5, 0.0);
  const moveit_msgs::msg::PositionConstraint pcm = createPlaneConstraint();

  moveit_msgs::msg::Constraints path_constraints;

  path_constraints.name = "use_equality_constraints";

  path_constraints.position_constraints.emplace_back(pcm);

  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(pose_goal);
  move_group_->setPathConstraints(path_constraints);

  planPathAndExecute();
}

void Faze4Motion::lineConstraintsDemo()
{
  tf2::Quaternion q;
  q.setRPY(M_PI / 2, M_PI, 0);
  moveToTarget(tf2::Vector3(-0.15, -0.49, 0.0), q);

  move_group_->setMaxVelocityScalingFactor(0.2);
  move_group_->setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(rclcpp::get_logger("Faze4Motion"), "lineConstraintsDemo");

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools_->publishText(
    text_pose, "lineConstraintsDemo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

  const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.4, 0.01, -0.01);
  const moveit_msgs::msg::PositionConstraint pcm = createLineConstraint();

  moveit_msgs::msg::Constraints path_constraints;

  path_constraints.name = "use_equality_constraints";

  path_constraints.position_constraints.emplace_back(pcm);

  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(pose_goal);
  move_group_->setPathConstraints(path_constraints);

  planPathAndExecute();

  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setMaxAccelerationScalingFactor(1.0);
}

void Faze4Motion::orientationConstraintsDemo()
{
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI / 2);
  moveToTarget(tf2::Vector3(-0.3, -0.5, 0.5), q);

  RCLCPP_INFO(rclcpp::get_logger("Faze4Motion"), "orientationConstraintsDemo");

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools_->publishText(
    text_pose, "orientationConstraintsDemo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

  const geometry_msgs::msg::PoseStamped pose_goal = createPoseGoal(0.0, 0.5, -0.3);

  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.header.frame_id = ref_link_;
  ocm.link_name = ee_link_;
  ocm.orientation = pose_goal.pose.orientation;
  ocm.absolute_x_axis_tolerance = 1.0;
  ocm.absolute_y_axis_tolerance = 1.0;
  ocm.absolute_z_axis_tolerance = 1.0;
  ocm.weight = 1.0;

  moveit_msgs::msg::Constraints cm;
  cm.name = "orientation constraints";
  cm.orientation_constraints.emplace_back(ocm);

  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(pose_goal);
  move_group_->setPathConstraints(cm);

  planPathAndExecute();
}

geometry_msgs::msg::PoseStamped Faze4Motion::createPoseGoal(double dx, double dy, double dz)
{
  geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

  pose.pose.position.x += dx;
  pose.pose.position.y += dy;
  pose.pose.position.z += dz;

  return pose;
}

moveit_msgs::msg::PositionConstraint Faze4Motion::createBoxConstraint()
{
  moveit_msgs::msg::PositionConstraint pcm;
  pcm.header.frame_id = ref_link_;
  pcm.link_name = ee_link_;
  pcm.weight = 1.0;

  shape_msgs::msg::SolidPrimitive cbox;
  cbox.type = shape_msgs::msg::SolidPrimitive::BOX;
  cbox.dimensions = {0.1, 0.4, 0.4};
  pcm.constraint_region.primitives.emplace_back(cbox);

  geometry_msgs::msg::Pose cbox_pose;
  cbox_pose.position.x = -0.3;
  cbox_pose.position.y = -0.35;
  cbox_pose.position.z = 0.35;
  pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

  visual_tools_->publishCuboid(
    cbox_pose, cbox.dimensions[0], cbox.dimensions[1], cbox.dimensions[2],
    rviz_visual_tools::TRANSLUCENT_LIGHT);

  return pcm;
}

moveit_msgs::msg::PositionConstraint Faze4Motion::createPlaneConstraint()
{
  moveit_msgs::msg::PositionConstraint pcm;
  pcm.header.frame_id = ref_link_;
  pcm.link_name = ee_link_;
  pcm.weight = 1.0;

  shape_msgs::msg::SolidPrimitive cbox;
  cbox.type = shape_msgs::msg::SolidPrimitive::BOX;

  cbox.dimensions = {2.0, 0.0005, 2.0};
  pcm.constraint_region.primitives.emplace_back(cbox);

  geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

  geometry_msgs::msg::Pose cbox_pose;
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, M_PI / 2);
  cbox_pose.orientation = tf2::toMsg(q);
  cbox_pose.position = pose.pose.position;

  pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

  visual_tools_->publishCuboid(
    cbox_pose, cbox.dimensions[0], cbox.dimensions[1], cbox.dimensions[2],
    rviz_visual_tools::TRANSLUCENT_LIGHT);
  visual_tools_->trigger();

  return pcm;
}

moveit_msgs::msg::PositionConstraint Faze4Motion::createLineConstraint()
{
  moveit_msgs::msg::PositionConstraint pcm;
  pcm.header.frame_id = ref_link_;
  pcm.link_name = ee_link_;
  pcm.weight = 1.0;

  shape_msgs::msg::SolidPrimitive cbox;
  cbox.type = shape_msgs::msg::SolidPrimitive::BOX;

  cbox.dimensions = {0.0005, 0.0005, 1.0};
  pcm.constraint_region.primitives.emplace_back(cbox);

  geometry_msgs::msg::PoseStamped pose = move_group_->getCurrentPose();

  geometry_msgs::msg::Pose cbox_pose;
  cbox_pose.position = pose.pose.position;

  // turn the constraint region 90 degrees around the y-axis and apply some corrections due to hardware
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI / 2 + 0.0250, 0.0250);
  cbox_pose.orientation = tf2::toMsg(q);

  pcm.constraint_region.primitive_poses.emplace_back(cbox_pose);

  visual_tools_->publishCuboid(
    cbox_pose, cbox.dimensions[0], cbox.dimensions[1], cbox.dimensions[2],
    rviz_visual_tools::TRANSLUCENT_DARK);  //To make it more visible
  visual_tools_->trigger();

  return pcm;
}
};  // namespace faze4_gui
