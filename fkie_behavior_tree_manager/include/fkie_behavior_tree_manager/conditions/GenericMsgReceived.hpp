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

#ifndef GENERIC_MSG_CONDITION_H
#define GENERIC_MSG_CONDITION_H

#include <fkie_behavior_tree_manager/logging.h>
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <topic_tools/shape_shifter.h>

namespace BehaviorTreeNodes
{
class GenericMsgReceived : public BT::ConditionNode
{
public:
  std::string name;
  ros::Subscriber sub;
  std::string parameter_topic;

  bool message_received;
  bool valid_init = false;

  GenericMsgReceived(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {};
    return ports;
  }

  BT::NodeStatus tick() override;

  void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name);
};

inline BT::NodeStatus GenericMsgReceived::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!valid_init)
    return BT::NodeStatus::FAILURE;

  if (message_received)
  {
    BT_ROS_INFO_STREAM(name << " New message received");
    message_received = false;
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}

GenericMsgReceived::GenericMsgReceived(const std::string& _name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(_name, config), name("[" + _name + "] ")
{
  ros::NodeHandle private_node("~");
  ros::NodeHandle public_node;

  std::string topic;

  parameter_topic = "GenericMsgReceived/" + _name + "_topic";

  private_node.param<std::string>(parameter_topic, topic, "msg_in");

  if (topic.empty())
  {
    BT_ROS_ERROR_STREAM(name << " [topic] is empty, check parameter [" << parameter_topic << "]");
    return;
  }

  // who is afraid of lambdas and boost::functions ?
  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
  callback = [&](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { topicCallback(msg, topic); };
  sub = public_node.subscribe(topic, 10, callback);

  valid_init = true;

  BT_ROS_INFO_STREAM(name << " initialized");
}

void GenericMsgReceived::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name)
{
  // BT_ROS_DEBUG_STREAM(name << " new message received in topic: [" << topic_name << "]");
  message_received = true;
}

}  // namespace BehaviorTreeNodes

#endif  // GENERIC_MSG_CONDITION_H