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

#ifndef RATE_CONTROLLER_NODE_H
#define RATE_CONTROLLER_NODE_H

#include "behaviortree_cpp_v3/decorator_node.h"
#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <string>

namespace BehaviorTreeNodes
{
class RateController : public BT::DecoratorNode
{
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period = 1.0;
  double hz = 1.0;
  bool valid_init = false;

public:
  std::string name;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<double>("hz") };
    return ports;
  }

  RateController(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::DecoratorNode(_name, config), name("[" + _name + "] ")
  {
    auto res = BT::TreeNode::getInput<double>("hz", hz);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << " error reading [hz]: " << res.error());
      valid_init = false;
      return;
    }

    period = 1.0 / hz;
    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    if (status() == BT::NodeStatus::IDLE)
    {
      // Reset the starting point (moving from IDLE to RUNNING)
      start_ = std::chrono::high_resolution_clock::now();
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Determine how long its been since we've started this iteration
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - start_;

    // Now, get that in seconds
    typedef std::chrono::duration<float> float_seconds;
    auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

    // If we've exceed the specified period, execute the child node
    if (seconds.count() >= period)
    {
      const BT::NodeStatus child_state = child_node_->executeTick();
      BT_ROS_DEBUG_STREAM(name << "Ticking node [" << child_node_->name() << "] - [" << seconds.count() << "]");

      switch (child_state)
      {
        case BT::NodeStatus::SUCCESS:
          // Reset the timer
          start_ = std::chrono::high_resolution_clock::now();
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::FAILURE:
          // Reset the timer
          start_ = std::chrono::high_resolution_clock::now();
          return BT::NodeStatus::FAILURE;

        default:
          // We'll try again next time
          return BT::NodeStatus::RUNNING;
      }
    }

    return status();
  }
};

}  // namespace BehaviorTreeNodes

#endif  // RATE_CONTROLLER_NODE_H