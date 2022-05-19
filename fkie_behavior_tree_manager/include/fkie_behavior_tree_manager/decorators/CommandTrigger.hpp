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

#ifndef SWITCH_ONCE_NODE_H
#define SWITCH_ONCE_NODE_H

#include "behaviortree_cpp_v3/decorator_node.h"
#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <string>

namespace BehaviorTreeNodes
{
class CommandTrigger : public BT::DecoratorNode
{
public:
  std::string name;
  ros::Subscriber sub_cmd;
  std::string cmd_on, topic_cmd;
  bool valid_init = false;
  bool msg_enable = false;
  bool normally_open = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports;
    return ports;
  }

  CommandTrigger(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::DecoratorNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;

    std::string parameter_topic_cmd = "CommandTrigger/" + _name + "_topic_cmd";
    std::string parameter_cmd_on = "CommandTrigger/" + _name + "_cmd_on";
    std::string parameter_normally_open = "CommandTrigger/" + _name + "_normally_open";

    private_node.param<std::string>(parameter_topic_cmd, topic_cmd, std::string(""));
    private_node.param<std::string>(parameter_cmd_on, cmd_on, std::string(""));
    private_node.param<bool>(parameter_normally_open, normally_open, true);

    if (topic_cmd.empty())
    {
      BT_ROS_ERROR_STREAM(name << "Parameter [" << parameter_topic_cmd << "] is empty.");
      return;
    }

    if (cmd_on.empty())
    {
      BT_ROS_ERROR_STREAM(name << "Parameter [" << parameter_cmd_on << "] is empty.");
      return;
    }

    sub_cmd = public_node.subscribe(topic_cmd, 2, &CommandTrigger::callbackCmd, this);
    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    if ((normally_open && msg_enable) || (!normally_open && !msg_enable))
    {
      const BT::NodeStatus child_state = child_node_->executeTick();
      BT_ROS_DEBUG_STREAM(name << "Ticking node [" << child_node_->name() << "]");
      msg_enable = false;  // prevent following executions
      return child_state;
    }
    else
    {
      msg_enable = false;  // prevent following executions
      return BT::NodeStatus::FAILURE;
    }
  }

  void callbackCmd(const std_msgs::String& msg)
  {
    if (msg.data == cmd_on)
      msg_enable = true;

    BT_ROS_DEBUG_STREAM(name << "new message received msg_enable: " << msg_enable);
  }
};

}  // namespace BehaviorTreeNodes

#endif  // SWITCH_ONCE_NODE_H