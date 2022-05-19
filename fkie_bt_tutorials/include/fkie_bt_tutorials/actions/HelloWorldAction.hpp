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

#ifndef HELLO_WORLD_ACTION_H
#define HELLO_WORLD_ACTION_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <ros/ros.h>

namespace BTTutorialGroup
{
class HelloWorldAction : public BT::SyncActionNode
{
public:
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<std::string>("message_in"), BT::OutputPort<std::string>("message_out") };

    return ports;
  }

  HelloWorldAction(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }

  ~HelloWorldAction() = default;

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    std::string message_in;
    auto res = BT::TreeNode::getInput("message_in", message_in);
    if (!res)
    {
      // if no message available, greet the world again :)
      message_in = "Hello World";

      // optionally, you can print error message and stop execution by returning [BT::NodeStatus::FAILURE]
      // BT_ROS_ERROR_STREAM("Could not load port [message_in]: " << res.error());
      // return BT::NodeStatus::FAILURE;
    }

    // print the message to console log and wait 2 secs.
    BT_ROS_INFO_STREAM("message_in: " << message_in);

    // Write something to the [OutputPort] called [message_out]:
    BT::TreeNode::setOutput("message_out", std::string("BT Tutorials: HelloWorldAction"));

    ros::Duration(2.0).sleep();

    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace BTTutorialGroup
#endif  // HELLO_WORLD_ACTION_H