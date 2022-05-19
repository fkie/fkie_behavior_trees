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

#ifndef CONDITIONAL_LOOP_H
#define CONDITIONAL_LOOP_H

#include "behaviortree_cpp_v3/decorator_node.h"
#include <fkie_behavior_tree_manager/logging.h>
#include <string>

namespace BehaviorTreeNodes
{
class ConditionalLoop : public BT::DecoratorNode
{
private:
  bool condition = false;

public:
  std::string name;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<bool>("condition") };
    return ports;
  }

  ConditionalLoop(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::DecoratorNode(_name, config), name("[" + _name + "] ")
  {
    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    auto res = BT::TreeNode::getInput<bool>("condition", condition);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << " Error reading [condition]: " << res.error());
      return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus loop_status = BT::NodeStatus::IDLE;

    while (ros::ok() && loop_status != BT::NodeStatus::FAILURE && condition)
    {
      // tick node and wait

      ros::spinOnce();

      BT_ROS_DEBUG_STREAM(name << "Ticking node [" << child_node_->name() << "]");
      const BT::NodeStatus child_state = child_node_->executeTick();

      switch (child_state)
      {
        case BT::NodeStatus::SUCCESS:
          loop_status = BT::NodeStatus::SUCCESS;
          break;

        case BT::NodeStatus::RUNNING:
          loop_status = BT::NodeStatus::RUNNING;
          break;

        case BT::NodeStatus::FAILURE:
          loop_status = BT::NodeStatus::FAILURE;
          break;

        default:
          loop_status = BT::NodeStatus::FAILURE;
          break;
      }

      res = BT::TreeNode::getInput<bool>("condition", condition);
      if (!res)
      {
        BT_ROS_ERROR_STREAM(name << " Error reading [condition]: " << res.error());
        return BT::NodeStatus::FAILURE;
      }

      if (condition)
      {
        BT_ROS_DEBUG_STREAM(name << "condition is true, ticking again the node.");
      }
    }

    return loop_status;
  }
};

}  // namespace BehaviorTreeNodes

#endif  // CONDITIONAL_LOOP_H