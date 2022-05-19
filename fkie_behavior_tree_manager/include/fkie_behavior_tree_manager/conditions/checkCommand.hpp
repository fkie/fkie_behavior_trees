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

#ifndef CHECK_COMMAND_H
#define CHECK_COMMAND_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

namespace BehaviorTreeNodes
{
BT::NodeStatus checkCommand(BT::TreeNode& self)
{
  std::string cmd;
  auto res = self.getInput("cmd", cmd);
  if (!res)
  {
    BT_ROS_ERROR_STREAM(self.name() << " Error at reading [cmd]: " << res.error());
    return BT::NodeStatus::FAILURE;
  }

  ros::NodeHandle private_node("~");
  std::string check_string_topic;
  float check_string_timeout = 2.0;
  private_node.param<std::string>("checkCommand/check_string_topic", check_string_topic, std::string(""));
  private_node.param<float>("checkCommand/check_string_timeout", check_string_timeout, 1.0f);

  if (check_string_topic.empty())
  {
    BT_ROS_ERROR_STREAM(self.name() << " check_string_topic is empty");
    return BT::NodeStatus::FAILURE;
  }

  // wait [timeout_target_pose] for first message
  std_msgs::StringConstPtr cmd_once =
      ros::topic::waitForMessage<std_msgs::String>(check_string_topic, ros::Duration(check_string_timeout));
  if (!cmd_once)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (cmd == cmd_once->data)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
};

}  // namespace BehaviorTreeNodes

#endif  // CHECK_COMMAND_H