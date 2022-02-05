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

#ifndef FAZE4_GUI__FAZE4_GUI_HPP_
#define FAZE4_GUI__FAZE4_GUI_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include "include/faze4_motion.hpp"

class QLineEdit;
class QComboBox;
class QCheckBox;
class QPushButton;
class QHBoxLayout;
class QWidget;

namespace faze4_gui
{
class Faze4Gui : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Faze4Gui(QWidget * parent = nullptr);

protected Q_SLOTS:
  void plan_execute();
  void demoModeSelected(int mode);

protected:
  QPushButton * plan_execute_btn_;
  QComboBox * demo_mode_selector_;

private:
  rclcpp::Node::SharedPtr node_;
  std::future<void> future_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  Faze4Motion * faze4_motion_;

  int demo_mode_selected_ = 0;
};

}  // namespace faze4_gui

#endif  // FAZE4_GUI__FAZE4_GUI_HPP_
