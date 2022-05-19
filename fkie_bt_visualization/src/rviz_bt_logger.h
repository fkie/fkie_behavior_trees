// Copyright 2022 Fraunhofer FKIE - All Rights Reserved
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

#ifndef BT_LOGGER_PANEL_H
#define BT_LOGGER_PANEL_H

#include "boost/date_time/posix_time/posix_time.hpp"
#include <QCheckBox>
#include <QDebug>
#include <QDomDocument>
#include <QFile>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QProcess>
#include <QPushButton>
#include <QTextBlock>
#include <QTextCursor>
#include <QTextEdit>
#include <QTimer>
#include <QTreeView>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>

#ifndef Q_MOC_RUN
#include "fkie_bt_visualization/TreeModel.hpp"
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include "randomcolor/randomcolor.h"
#include <fkie_behavior_tree_msgs/BTLogging.h>

class QLineEdit;

namespace fkie_bt_visualization
{
class BTLoggerPanel : public rviz::Panel
{
  Q_OBJECT

public:
  BTLoggerPanel(QWidget* parent = 0);

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

public Q_SLOTS:

  void updateTopicName();
  void setTopicName(const QString& bt_name);

  void clearButton();
  void updateScrollToEnd(bool state);

protected Q_SLOTS:

  // ROS Callbacks
  void callbackBTlogging(const fkie_behavior_tree_msgs::BTLoggingConstPtr& msg);
  void rosSpin();

protected:
  ros::Subscriber sub_bt_log;
  QString bt_current_logging_topic_;

  // GUI Elements
  QTextEdit* q_text;
  QLineEdit* output_bt_topic_editor_;
  QString bt_topic_logging;

  QCheckBox* q_check_info;
  QCheckBox* q_check_warn;
  QCheckBox* q_check_error;
  QCheckBox* q_check_fatal;
  QCheckBox* q_check_scroll_to_end;
  bool scroll_to_end = true;

  QPushButton* q_button_clear;

  // ROS Elements
  ros::Subscriber sub_bt_status;
  ros::Subscriber sub_bt_node_status;

  ros::NodeHandle nh_;

  std::map<std::string, std::string> map_color_sources;
  std::string generateRandomColor();
  int counter = 0;
};
}  // namespace fkie_bt_visualization

#endif  // BT_LOGGER_PANEL_H
