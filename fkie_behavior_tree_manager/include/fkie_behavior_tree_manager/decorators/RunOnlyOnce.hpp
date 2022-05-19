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

#ifndef RUN_ONLY_ONCE_H
#define RUN_ONLY_ONCE_H

#include "behaviortree_cpp_v3/decorator_node.h"
#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <string>

namespace BehaviorTreeNodes
{
/*
RunOnlyOnce: This decorator ticks its children only once, useful for initializing states/poses.
*/
class RunOnlyOnce : public BT::DecoratorNode
{
public:
  std::string name;
  bool already_executed = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports;
    return ports;
  }

  RunOnlyOnce(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::DecoratorNode(_name, config), name("[" + _name + "] ")
  {
    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (already_executed)
      return BT::NodeStatus::SUCCESS;

    BT_ROS_DEBUG_STREAM(name << "Ticking node [" << child_node_->name() << "]");
    const BT::NodeStatus child_state = child_node_->executeTick();

    if (child_state == BT::NodeStatus::FAILURE)
    {
      BT_ROS_WARN_STREAM(name << "Node [" << child_node_->name() << "] returns FAILURE, will be ticked again!");
    }
    else
    {
      already_executed = true;
    }

    return child_state;
  }
};

}  // namespace BehaviorTreeNodes

#endif  // RUN_ONLY_ONCE_H