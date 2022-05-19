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

#ifndef DELAY_ACTION_NODE_H
#define DELAY_ACTION_NODE_H

#include <behaviortree_cpp_v3/decorator_node.h>
#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <string>

namespace BehaviorTreeNodes
{
class DelayAction : public BT::SyncActionNode
{
public:
  std::string name;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<Pose2D>("delay_time") };
    return ports;
  }

  DelayAction(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(_name, config), name("[" + _name + "] ")
  {
    BT_ROS_INFO_STREAM(name << " initialized");
  }

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    double delay_time = 0.0;
    auto res = BT::TreeNode::getInput("delay_time", delay_time);
    if (!res)
    {
      ROS_ERROR_STREAM("DelayAction: Could not load delay_time, resetting variable. Error: " << res.error());
      BT::TreeNode::setOutput("delay_time", 0.0);
      return BT::NodeStatus::FAILURE;
    }

    BT_ROS_INFO_STREAM(name << ": Waiting for: " << delay_time << " (s)");

    ros::Duration(delay_time).sleep();
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace BehaviorTreeNodes

#endif  // DELAY_ACTION_NODE_H