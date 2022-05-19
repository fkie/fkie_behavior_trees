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

#include "rviz_bt_status.h"

namespace fkie_bt_visualization
{
BTStatusPanel::BTStatusPanel(QWidget* parent) : rviz::Panel(parent)
{
  q_tree_view = std::make_unique<QTreeView>();

  q_tree_view_compact = new QTreeView();

  // Create plugin layout
  QHBoxLayout* bt_name_layout = new QHBoxLayout;

  QLabel* q_bt_name = new QLabel("Node:");
  q_bt_name->setToolTip("Name of the ROS node running the Behavior Tree Manager");
  bt_name_layout->addWidget(q_bt_name);

  output_bt_name_editor_ = new QLineEdit;
  output_bt_name_editor_->setToolTip("Name of the ROS node running the Behavior Tree Manager");
  bt_name_layout->addWidget(output_bt_name_editor_);
  connect(output_bt_name_editor_, SIGNAL(editingFinished()), this, SLOT(updateBTName()));

  q_bt_status = new QLabel("---");
  q_bt_status->setToolTip("Current Behavior Tree execution state");
  bt_name_layout->addWidget(q_bt_status);

  QVBoxLayout* current_action_layout = new QVBoxLayout;
  qlabel_action = new QLabel("---");
  qlabel_action->setToolTip("Last reported node and its execution state");
  current_action_layout->addWidget(qlabel_action);
  current_action_layout->addWidget(q_tree_view_compact);

  QHBoxLayout* control_layout = new QHBoxLayout;
  m_start_button = new QPushButton("Start", this);
  m_start_button->setToolTip("Start execution of the Behavior Tree");
  m_start_button->setHidden(true);
  control_layout->addWidget(m_start_button);
  connect(m_start_button, SIGNAL(released()), this, SLOT(startButton()));

  m_stop_button = new QPushButton("Stop", this);
  m_stop_button->setToolTip("Stop execution of the Behavior Tree. It will wait until current node stops.");
  m_stop_button->setHidden(true);
  control_layout->addWidget(m_stop_button);
  connect(m_stop_button, SIGNAL(released()), this, SLOT(stopButton()));

  m_tree_button = new QPushButton("Show", this);
  m_tree_button->setToolTip("Open current Tree");
  m_tree_button->setHidden(true);
  control_layout->addWidget(m_tree_button);
  connect(m_tree_button, SIGNAL(released()), this, SLOT(treeButton()));

  cb_mode = new QCheckBox("Compact", this);
  cb_mode->setChecked(use_compact_mode);
  connect(cb_mode, SIGNAL(clicked(bool)), this, SLOT(cbStateChanged(bool)));
  control_layout->addWidget(cb_mode);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(bt_name_layout);
  layout->addLayout(control_layout);
  layout->addLayout(current_action_layout);
  setLayout(layout);

  applyViewMode(use_compact_mode);

  // Start the timer: define looping rate
  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(rosSpin()));
  output_timer->start(200);
}

void BTStatusPanel::updateBTName()
{
  setBTName(output_bt_name_editor_->text());
}

void BTStatusPanel::startButton()
{
  pub_start.publish(std_msgs::Header());
}

void BTStatusPanel::stopButton()
{
  pub_stop.publish(std_msgs::Header());
}

void BTStatusPanel::cbStateChanged(bool)
{
  use_compact_mode = cb_mode->isChecked();
  applyViewMode(use_compact_mode);
}

void BTStatusPanel::treeButton()
{
  if (loadModels(current_bt_status))
  {
    q_tree_view->setModel(tree_model.getModel());
    q_tree_view->setSelectionMode(QAbstractItemView::SingleSelection);

    q_tree_view->resizeColumnToContents(0);
    q_tree_view->resizeColumnToContents(1);
    q_tree_view->resizeColumnToContents(2);

    std::vector<int> cs = tree_model.getColumnSize();
    // q_tree_view->setFixedWidth(cs[0] + cs[1] + cs[2]);
    q_tree_view->setMinimumWidth(cs[0] + cs[1] + cs[2]);
    q_tree_view->setMinimumHeight(tree_model.getRowCount() * 20);

    std::string title_name = "BT Viewer for: " + output_bt_name_editor_->text().toStdString();
    q_tree_view->setWindowTitle(QString(title_name.c_str()));

    q_tree_view->show();
  }
}

bool BTStatusPanel::loadModels(const fkie_behavior_tree_msgs::BTStatus& bt_status)
{
  // clear existing model
  tree_model.clearModels();

  // Load model according to BT status message
  bool success_load_tree = false;
  if (!bt_status.tree_file.empty())
    success_load_tree = tree_model.createBehaviorTreeModelFromFile(bt_status.tree_file);

  if (!bt_status.tree_xml.empty())
    success_load_tree = tree_model.createBehaviorTreeModelFromText(QString(bt_status.tree_xml.c_str()));

  if (!success_load_tree || !tree_model.isValid())
  {
    QMessageBox::warning(this, tr("Behavior Tree Plugin"),
                         tr("Could not load the Tree, check console log for details."));
  }

  return success_load_tree;
}

void BTStatusPanel::applyViewMode(const bool use_compact_mode)
{
  if (use_compact_mode)
  {
    // only shows current action
    qlabel_action->setHidden(false);
    q_tree_view_compact->setHidden(true);
    // this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    this->update();
  }
  else
  {
    // shows compact tree
    qlabel_action->setHidden(true);
    q_tree_view_compact->setHidden(false);
    this->layout()->setSizeConstraint(QLayout::SetNoConstraint);
    this->update();
  }
}

void BTStatusPanel::setBTName(const QString& new_bt_name)
{
  // Only take action if the name has changed.
  if (new_bt_name != bt_status_bt_name_)
  {
    bt_status_bt_name_ = new_bt_name;
    // If the bt_name is the empty string, don't publish anything.
    if (bt_status_bt_name_ == "")
    {
      sub_bt_status.shutdown();
      sub_bt_node_status.shutdown();
      pub_start.shutdown();
      pub_stop.shutdown();
    }
    else
    {
      tree_model.clearModels();
      q_bt_status->setText(QString::fromUtf8("---"));
      qlabel_action->setText(QString::fromUtf8("---"));

      sub_bt_status = nh_.subscribe<fkie_behavior_tree_msgs::BTStatus>(bt_status_bt_name_.toStdString() + "/bt_status",
                                                                       10, &BTStatusPanel::callbackBTStatus, this);
      sub_bt_node_status = nh_.subscribe<fkie_behavior_tree_msgs::BTNodeStatus>(
          bt_status_bt_name_.toStdString() + "/bt_node_status", 10, &BTStatusPanel::callbackBTNodeStatus, this);

      pub_start = nh_.advertise<std_msgs::Header>(bt_status_bt_name_.toStdString() + "/start_tree", 5, false);
      pub_stop = nh_.advertise<std_msgs::Header>(bt_status_bt_name_.toStdString() + "/stop_tree", 5, false);
    }

    applyViewMode(use_compact_mode);

    Q_EMIT configChanged();
  }
}

void BTStatusPanel::callbackBTStatus(const fkie_behavior_tree_msgs::BTStatusConstPtr& msg)
{
  std::string bt_text;

  if (msg->tree_is_running)
    bt_text = "<font color=green>Running";
  else
    bt_text = "<font color=red>Stopped";

  q_bt_status->setText(QString::fromUtf8(bt_text.c_str()));

  // Enable GUI elements:
  m_start_button->setHidden(false);
  m_stop_button->setHidden(false);
  m_tree_button->setHidden(false);

  // update QTreeView
  if (current_bt_status.tree_file != msg->tree_file || current_bt_status.tree_xml != msg->tree_xml)
  {
    if (loadModels(*msg))
    {
      q_tree_view_compact->setModel(tree_model.getCompactModel());
      q_tree_view_compact->setSelectionMode(QAbstractItemView::SingleSelection);
      q_tree_view_compact->setMinimumHeight(tree_model.getRowCount() * 17);

      q_tree_view_compact->resizeColumnToContents(0);

      std::vector<int> cs = tree_model.getColumnSizeCompact();
      q_tree_view_compact->setColumnWidth(0, cs[0]);
      q_tree_view_compact->setFixedWidth(cs[0]);
      // q_tree_view_compact->setMinimumWidth(q_tree_view_compact->width());
    }
  }

  current_bt_status = *msg;
}

void BTStatusPanel::callbackBTNodeStatus(const fkie_behavior_tree_msgs::BTNodeStatusConstPtr& msg)
{
  // Set current action
  std::string bt_action = "";

  bt_action += "[" + msg->node_type + "]<strong>  " + msg->node_name + "</strong>";
  bt_action += "<br>ID: " + msg->node_id;

  if (msg->node_status == "RUNNING")
  {
    bt_action += "<br>Status: <font color=darkorange>" + msg->node_status;
  }
  else if (msg->node_status == "FAILURE")
  {
    bt_action += "<br>Status: <font color=red>" + msg->node_status;
  }
  else
  {
    bt_action += "<br>Status: <font color=black>" + msg->node_status;
  }

  qlabel_action->setText(QString::fromUtf8(bt_action.c_str()));

  current_node_status = *msg;

  // Update model if available
  tree_model.setActiveAction(TreeItem(msg->node_type, msg->node_id, msg->node_name));
}

void BTStatusPanel::rosSpin()
{
  ros::spinOnce();
}

void BTStatusPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("BTName", bt_status_bt_name_);
}

// Load all configuration data for this panel from the given Config object.
void BTStatusPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  QString bt_name;
  if (config.mapGetString("BTName", &bt_name))
  {
    output_bt_name_editor_->setText(bt_name);

    // resize Line Edit To Contents
    QFontMetrics fm = output_bt_name_editor_->fontMetrics();
    int w_name_editor = fm.boundingRect(output_bt_name_editor_->text()).width();
    output_bt_name_editor_->resize(w_name_editor, output_bt_name_editor_->height());

    fm = q_bt_status->fontMetrics();
    int w_bt_status = fm.boundingRect(q_bt_status->text()).width();

    this->setMinimumWidth(w_name_editor + w_bt_status);

    updateBTName();
  }
}
}  // namespace fkie_bt_visualization

PLUGINLIB_EXPORT_CLASS(fkie_bt_visualization::BTStatusPanel, rviz::Panel)
