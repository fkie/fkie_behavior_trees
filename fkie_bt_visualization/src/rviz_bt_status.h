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

#ifndef BT_STATUS_PANEL_H
#define BT_STATUS_PANEL_H

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

#include <fkie_behavior_tree_msgs/BTNodeStatus.h>
#include <fkie_behavior_tree_msgs/BTStatus.h>

class QLineEdit;

namespace fkie_bt_visualization
{
class BTStatusPanel : public rviz::Panel
{
  Q_OBJECT

public:
  BTStatusPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

  void setBTName(const QString& bt_name);

protected Q_SLOTS:

  // ROS Callbacks
  void callbackBTStatus(const fkie_behavior_tree_msgs::BTStatusConstPtr& msg);
  void callbackBTNodeStatus(const fkie_behavior_tree_msgs::BTNodeStatusConstPtr& msg);
  void rosSpin();

  void updateBTName();

  // Button callbacks
  void startButton();
  void stopButton();
  void cbStateChanged(bool);
  void treeButton();

  bool loadModels(const fkie_behavior_tree_msgs::BTStatus& bt_status);
  void applyViewMode(const bool use_compact_mode);

protected:
  // GUI Elements
  QLineEdit* output_bt_name_editor_;
  QString bt_status_bt_name_;

  QLabel* qlabel_bt_status;
  QLabel* qlabel_action;
  QLabel* q_bt_status;

  QPushButton* m_start_button;
  QPushButton* m_stop_button;
  QPushButton* m_tree_button;

  QCheckBox* cb_mode;

  QTreeView* q_tree_view_compact;

  // ROS Elements
  ros::Subscriber sub_bt_status;
  ros::Subscriber sub_bt_node_status;

  ros::NodeHandle nh_;

  ros::Publisher pub_start;
  ros::Publisher pub_stop;

  fkie_behavior_tree_msgs::BTStatus current_bt_status;
  fkie_behavior_tree_msgs::BTNodeStatus current_node_status;

  std::unique_ptr<QTreeView> q_tree_view;
  TreeModel tree_model;

  bool use_compact_mode = true;
};
}  // namespace fkie_bt_visualization

#endif  // BT_STATUS_PANEL_H
