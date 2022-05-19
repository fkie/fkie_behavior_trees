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

#ifndef COMMAND_SWITCH_NODE_H
#define COMMAND_SWITCH_NODE_H

#include <behaviortree_cpp_v3/decorator_node.h>
#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <string>

namespace BehaviorTreeNodes
{
class CommandSwitch : public BT::DecoratorNode
{
public:
  std::string name;
  ros::Subscriber sub_cmd;
  std::string cmd_on, cmd_off, topic_cmd;
  bool valid_init = false;
  bool gate_enable = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports;
    return ports;
  }

  CommandSwitch(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::DecoratorNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;

    std::string parameter_topic_cmd = "CommandSwitch/" + _name + "_topic_cmd";
    std::string parameter_cmd_on = "CommandSwitch/" + _name + "_cmd_on";
    std::string parameter_cmd_off = "CommandSwitch/" + _name + "_cmd_off";

    private_node.param<std::string>(parameter_topic_cmd, topic_cmd, std::string(""));
    private_node.param<std::string>(parameter_cmd_on, cmd_on, std::string(""));
    private_node.param<std::string>(parameter_cmd_off, cmd_off, std::string(""));

    if (topic_cmd.empty())
    {
      BT_ROS_ERROR_STREAM(name << "Parameter [" << parameter_topic_cmd << "] is empty.");
      return;
    }

    if (cmd_on.empty())
    {
      ROS_ERROR_STREAM(name << "Parameter [" << parameter_cmd_on << "] is empty.");
      return;
    }

    if (cmd_off.empty())
    {
      ROS_ERROR_STREAM(name << "Parameter [" << parameter_cmd_off << "] is empty.");
      return;
    }

    sub_cmd = public_node.subscribe(topic_cmd, 2, &CommandSwitch::callbackCmd, this);
    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    if (gate_enable)
    {
      const BT::NodeStatus child_state = child_node_->executeTick();
      BT_ROS_DEBUG_STREAM(name << "Ticking node [" << child_node_->name() << "]");
      return child_state;
    }
    else
      return BT::NodeStatus::FAILURE;
  }

  void callbackCmd(const std_msgs::String& msg)
  {
    if (msg.data == cmd_on)
      gate_enable = true;

    if (msg.data == cmd_off)
      gate_enable = false;

    BT_ROS_DEBUG_STREAM(name << " New message received, gate_enable: " << gate_enable);
  }
};

}  // namespace BehaviorTreeNodes

#endif  // COMMAND_SWITCH_NODE_H