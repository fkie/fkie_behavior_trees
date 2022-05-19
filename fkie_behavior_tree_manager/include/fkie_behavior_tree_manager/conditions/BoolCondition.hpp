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

#ifndef BOOL_CONDITION_H
#define BOOL_CONDITION_H

#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <fkie_behavior_tree_manager/logging.h>

namespace BehaviorTreeNodes
{
class BoolCondition : public BT::ConditionNode
{
public:
  std::string name;
  ros::Subscriber sub_bool;
  std::string topic_bool;
  std_msgs::Bool current_bool;

  std::string parameter_topic_bool;
  std::string parameter_latch_mode;
  bool latch_mode;

  bool valid_init = false;

  BoolCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {};
    return ports;
  }

  BT::NodeStatus tick() override;
  void callbackBool(const std_msgs::Bool& msg);
};

inline BT::NodeStatus BoolCondition::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!valid_init)
    return BT::NodeStatus::FAILURE;

  BT_ROS_DEBUG_STREAM(name << "  current_bool: " << (current_bool.data ? "true" : "false"));
  if (current_bool.data)
  {
    if (!latch_mode)
    {
      BT_ROS_DEBUG_STREAM(name << "  setting current_bool to false. (latch_mode disabled)");
      current_bool.data = false;
    }
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}

BoolCondition::BoolCondition(const std::string& _name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(_name, config), name("[" + _name + "] ")
{
  ros::NodeHandle private_node("~");
  ros::NodeHandle public_node;

  parameter_topic_bool = "BoolCondition/" + _name + "_topic_bool";
  parameter_latch_mode = "BoolCondition/" + _name + "_latch_mode";

  private_node.param<std::string>(parameter_topic_bool, topic_bool, "bool_in");
  private_node.param<bool>(parameter_latch_mode, latch_mode, false);

  if (topic_bool.empty())
  {
    BT_ROS_ERROR_STREAM(name << " [topic_bool] is empty, check parameter [" << parameter_topic_bool << "]");
    return;
  }

  sub_bool = public_node.subscribe(topic_bool, 1, &BoolCondition::callbackBool, this);
  valid_init = true;

  BT_ROS_INFO_STREAM(name << " initialized");
}

void BoolCondition::callbackBool(const std_msgs::Bool& msg)
{
  current_bool = msg;
  BT_ROS_DEBUG_STREAM(name << " New bool message received: " << current_bool.data);
}

}  // namespace BehaviorTreeNodes

#endif  // BOOL_CONDITION_H
