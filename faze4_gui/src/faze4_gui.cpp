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

#include "include/faze4_gui.hpp"
#include "include/faze4_motion.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>

namespace faze4_gui
{
Faze4Gui::Faze4Gui(QWidget * parent) : rviz_common::Panel(parent)
{
  plan_execute_btn_ = new QPushButton("Plan && Execute");
  connect(plan_execute_btn_, SIGNAL(clicked()), this, SLOT(plan_execute()));

  demo_mode_selector_ = new QComboBox(this);
  demo_mode_selector_->addItem("Box Constraints Demo");
  demo_mode_selector_->addItem("Plane Constraints Demo");
  demo_mode_selector_->addItem("Line Constraints Demo");
  demo_mode_selector_->addItem("Orientation Constraints Demo");
  demo_mode_selector_->addItem("All demos in sequence");
  demo_mode_selector_->addItem("Debug");
  demo_mode_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  demo_mode_selector_->setToolTip("Select which demo to run");
  connect(demo_mode_selector_, SIGNAL(activated(int)), this, SLOT(demoModeSelected(int)));

  // Main layout
  auto * main_layout = new QHBoxLayout(this);
  main_layout->addWidget(plan_execute_btn_);
  main_layout->addWidget(demo_mode_selector_);

  node_ = std::make_shared<rclcpp::Node>("faze4_motion");
  executor_.add_node(node_);

  std::thread node_thread([this]() { this->executor_.spin(); });
  node_thread.detach();

  faze4_motion_ = new Faze4Motion(node_);
}

void Faze4Gui::plan_execute()
{
  RCLCPP_INFO(rclcpp::get_logger("Faze4Gui"), "Plan and execute");

  future_ = std::async(std::launch::async, [this]() {
    this->plan_execute_btn_->setEnabled(false);
    switch (demo_mode_selected_)
    {
      case 0:
        this->faze4_motion_->boxConstraintsDemo();
        break;
      case 1:
        this->faze4_motion_->planeConstraintsDemo();
        break;
      case 2:
        this->faze4_motion_->lineConstraintsDemo();
        break;
      case 3:
        this->faze4_motion_->orientationConstraintsDemo();
        break;
      case 4:
        this->faze4_motion_->boxConstraintsDemo();
        this->faze4_motion_->planeConstraintsDemo();
        this->faze4_motion_->lineConstraintsDemo();
        this->faze4_motion_->orientationConstraintsDemo();
        break;
      case 5:
        this->faze4_motion_->debug();
      default:
        break;
    }
    this->plan_execute_btn_->setEnabled(true);
  });
}

void Faze4Gui::demoModeSelected(int demo) { demo_mode_selected_ = demo; }

}  // namespace faze4_gui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(faze4_gui::Faze4Gui, rviz_common::Panel)
