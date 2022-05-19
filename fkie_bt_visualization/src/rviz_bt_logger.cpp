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

#include "rviz_bt_logger.h"

namespace fkie_bt_visualization
{
BTLoggerPanel::BTLoggerPanel(QWidget* parent) : rviz::Panel(parent)
{
  QHBoxLayout* layout = new QHBoxLayout;

  QVBoxLayout* bt_header_layout = new QVBoxLayout;

  QHBoxLayout* l_topic = new QHBoxLayout;

  QLabel* q_bt_topic = new QLabel("Topic:");
  q_bt_topic->setMaximumWidth(45);
  q_bt_topic->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  q_bt_topic->setToolTip("Topic [bt_logging] from the Behavior Tree Manager");
  l_topic->addWidget(q_bt_topic);
  l_topic->setAlignment(q_bt_topic, Qt::AlignLeft);

  output_bt_topic_editor_ = new QLineEdit;
  output_bt_topic_editor_->setToolTip("Topic used for logging. Ex. /bt_logging ");
  output_bt_topic_editor_->setText(bt_current_logging_topic_);
  output_bt_topic_editor_->setMinimumWidth(200);
  output_bt_topic_editor_->setMaximumWidth(200);
  output_bt_topic_editor_->setSizePolicy(QSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed));
  l_topic->addWidget(output_bt_topic_editor_);
  l_topic->setAlignment(output_bt_topic_editor_, Qt::AlignLeft);
  connect(output_bt_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopicName()));

  QHBoxLayout* l_level = new QHBoxLayout;

  QFont font;  //("Arial", 10, QFont::Normal);
  font.setPointSize(10);

  q_check_info = new QCheckBox("INFO");
  q_check_info->setChecked(true);
  q_check_info->setFont(font);
  l_level->addWidget(q_check_info);

  q_check_warn = new QCheckBox("WARN");
  q_check_warn->setChecked(true);
  q_check_warn->setFont(font);
  l_level->addWidget(q_check_warn);

  q_check_error = new QCheckBox("ERROR");
  q_check_error->setChecked(true);
  q_check_error->setFont(font);
  l_level->addWidget(q_check_error);

  q_check_fatal = new QCheckBox("FATAL");
  q_check_fatal->setChecked(true);
  q_check_fatal->setFont(font);
  l_level->addWidget(q_check_fatal);

  bt_header_layout->addLayout(l_topic);
  bt_header_layout->addLayout(l_level);

  QHBoxLayout* l_options = new QHBoxLayout;

  q_button_clear = new QPushButton("Clear Text");
  l_options->addWidget(q_button_clear);
  l_topic->setAlignment(q_button_clear, Qt::AlignLeft);
  connect(q_button_clear, SIGNAL(released()), this, SLOT(clearButton()));

  q_check_scroll_to_end = new QCheckBox("Scroll to end");
  q_check_scroll_to_end->setChecked(true);
  l_options->addWidget(q_check_scroll_to_end);
  l_topic->setAlignment(q_check_scroll_to_end, Qt::AlignLeft);
  connect(q_check_scroll_to_end, SIGNAL(clicked(bool)), this, SLOT(updateScrollToEnd(bool)));

  bt_header_layout->addLayout(l_options);

  layout->addLayout(bt_header_layout);

  q_text = new QTextEdit();
  layout->addWidget(q_text);

  setLayout(layout);

  // Start the timer: define looping rate
  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(rosSpin()));
  output_timer->start(50);
}

void BTLoggerPanel::callbackBTlogging(const fkie_behavior_tree_msgs::BTLoggingConstPtr& msg)
{
  std::string html_color;

  if (map_color_sources.count(msg->source) == 0)
  {
    map_color_sources.insert(std::make_pair(msg->source, generateRandomColor()));
    qDebug("BTLoggerPanel: generating color for %s: %s", msg->source.c_str(),
           map_color_sources.at(msg->source).c_str());
  }

  if (q_check_info->isChecked() && msg->level == "INFO")
  {
    html_color = "black";
  }
  else if (q_check_warn->isChecked() && msg->level == "WARN")
  {
    html_color = "orange";
  }
  else if (q_check_error->isChecked() && msg->level == "ERROR")
  {
    html_color = "red";
  }
  else if (q_check_fatal->isChecked() && msg->level == "FATAL")
  {
    html_color = "red";
  }
  else
    return;

  std::string t;
  t += "<span style='color:" + map_color_sources.at(msg->source) + "'>[" + msg->source + "]</span>";
  t += "<span style='color:" + html_color + "'> [" + msg->level + "] " + msg->data + "</span>";
  QString msg_append = QString::fromUtf8(t.c_str());
  q_text->append(msg_append);

  if (scroll_to_end)
  {
    q_text->moveCursor(QTextCursor::End);
    q_text->ensureCursorVisible();
  }
}

void BTLoggerPanel::rosSpin()
{
  ros::spinOnce();
}

void BTLoggerPanel::updateTopicName()
{
  setTopicName(output_bt_topic_editor_->text());
}

void BTLoggerPanel::setTopicName(const QString& new_bt_name)
{
  // Only take action if the name has changed.
  if (new_bt_name != bt_current_logging_topic_)
  {
    bt_current_logging_topic_ = new_bt_name;

    // If the bt_name is the empty string, don't publish anything.
    if (bt_current_logging_topic_ == "")
    {
      sub_bt_log.shutdown();
    }
    else
    {
      sub_bt_log = nh_.subscribe<fkie_behavior_tree_msgs::BTLogging>(bt_current_logging_topic_.toStdString(), 30,
                                                                     &BTLoggerPanel::callbackBTlogging, this);
    }
    Q_EMIT configChanged();
  }
}

void BTLoggerPanel::clearButton()
{
  q_text->setText("");
}

void BTLoggerPanel::updateScrollToEnd(bool state)
{
  scroll_to_end = state;

  QString msg_append;

  if (scroll_to_end)
    msg_append = QString::fromUtf8("[scroll_to_end] enabled");
  else
    msg_append = QString::fromUtf8("[scroll_to_end] disabled");

  q_text->append(msg_append);
}

void BTLoggerPanel::save(rviz::Config config) const
{
  qDebug("BTLoggerPanel: save");
  rviz::Panel::save(config);
  config.mapSetValue("bt_topic_logging", output_bt_topic_editor_->text());
}

// Load all configuration data for this panel from the given Config object.
void BTLoggerPanel::load(const rviz::Config& config)
{
  qDebug("BTLoggerPanel: Load");

  rviz::Panel::load(config);

  config.mapGetString("bt_topic_logging", &bt_topic_logging);  // update value if required

  qDebug("BTLoggerPanel: bt_topic_logging: [%s]", bt_topic_logging.toStdString().c_str());

  output_bt_topic_editor_->setText(bt_topic_logging);

  updateTopicName();
}

std::string BTLoggerPanel::generateRandomColor()
{
  auto ColorGeneratorNormal = [=]() -> std::function<std::tuple<int, int, int>()> {
    srand((int)time(NULL));
    RandomColor::Options o;
    o.hue = 0;
    o.hue_name = RandomColor::UNSET;
    o.seed = rand() % 1000;
    auto RG = RandomColor::RandomColorGenerator(o);
    int j = 3;
    int i = 1;
    return [i, j, RG]() mutable {
      bool flag_done = false;
      while (!flag_done)
      {
        i++;
        if (i > 8)
        {
          i = 2;
          j++;
          if (j > 3)
          {
            j = 1;
          }
        }
        flag_done = true;
        // auto color_name = static_cast<RandomColor::HUENAMES>(i);
        RG.options.luminosity = static_cast<RandomColor::LUMINOSITY>(j);
        RG.options.hue_name = static_cast<RandomColor::HUENAMES>(i);
      }
      return RG.randomColorRGB();
    };
  };

  // auto ColorGeneratorDeuteranopia = [=]() -> std::function<std::tuple<int, int, int>()> {
  //   srand((int)time(NULL));
  //   RandomColor::Options o;
  //   o.hue = 0;
  //   o.hue_name = RandomColor::UNSET;
  //   o.seed = rand() % 1000;
  //   auto RG = RandomColor::RandomColorGenerator(o);
  //   int j = 3;
  //   int i = 1;
  //   return [i, j, RG]() mutable {
  //     bool flag_done = false;
  //     while (!flag_done)
  //     {
  //       i++;
  //       if (i > 8)
  //       {
  //         i = 2;
  //         j++;
  //         if (j > 3)
  //         {
  //           j = 1;
  //         }
  //       }
  //       flag_done = true;
  //       auto color_name = static_cast<RandomColor::HUENAMES>(i);
  //       if (color_name == RandomColor::HUENAMES::RED || color_name == RandomColor::HUENAMES::ORANGE || color_name ==
  //       RandomColor::HUENAMES::GREEN || color_name == RandomColor::HUENAMES::PURPLE || color_name ==
  //       RandomColor::HUENAMES::PINK)
  //       {
  //         flag_done = false;
  //       }
  //       RG.options.luminosity = static_cast<RandomColor::LUMINOSITY>(j);
  //       RG.options.hue_name = static_cast<RandomColor::HUENAMES>(i);
  //     }
  //     return RG.randomColorRGB();
  //   };
  // };

  // auto G = ColorGeneratorDeuteranopia();
  auto G = ColorGeneratorNormal();

  std::string color;
  for (int i = 0; i <= counter; i++)
  {
    std::tuple<int, int, int> r = G();
    color = "rgb(" + std::to_string(std::get<0>(r)) + ", " + std::to_string(std::get<1>(r)) + ", " +
            std::to_string(std::get<2>(r)) + ")";
  }
  counter++;

  return color;
}
}  // namespace fkie_bt_visualization

PLUGINLIB_EXPORT_CLASS(fkie_bt_visualization::BTLoggerPanel, rviz::Panel)
