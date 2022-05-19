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

#ifndef CHECK_TIMEOUT_NODE_H
#define CHECK_TIMEOUT_NODE_H

#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <tf2_ros/transform_listener.h>

namespace BehaviorTreeNodes
{
class CheckTimeoutAsync : public BT::CoroActionNode
{
public:
  std::string name;

  double timeout = 0.0;

  ros::Time start;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<double>("timeout") };
    return ports;
  }

  CheckTimeoutAsync(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::CoroActionNode(_name, config), name("[" + _name + "] ")
  {
    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    BT::TreeNode::getInput("timeout", timeout);

    if (timeout <= 0)
    {
      // disable timeout, always running...
      return BT::NodeStatus::RUNNING;
    }

    // start new timer
    if (status() != BT::NodeStatus::RUNNING)
    {
      start = ros::Time::now();
      BT_ROS_INFO_STREAM(name << " starting timer, timeout configured to: " << timeout << " s");
      return BT::NodeStatus::RUNNING;
    }

    // Determine how long its been since we've started this iteration
    ros::Duration elapsed = ros::Time::now() - start;

    if (elapsed.toSec() < timeout)
    {
      // timer running, not timeout yet
      return BT::NodeStatus::RUNNING;
    }

    // timeout reached
    BT_ROS_INFO_STREAM(name << " timeout reached: " << elapsed.toSec());
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    BT_ROS_DEBUG_STREAM(name << "node has been halted!");
    setStatus(BT::NodeStatus::FAILURE);
    CoroActionNode::halt();
  }
};

}  // namespace BehaviorTreeNodes

#endif  // CHECK_TIMEOUT_NODE_H