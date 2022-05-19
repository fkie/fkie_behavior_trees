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

#ifndef WAIT_FOR_GENERIC_MSG_ASYNC_ACTION_NODE_H
#define WAIT_FOR_GENERIC_MSG_ASYNC_ACTION_NODE_H

#include <chrono>
#include <string>

#include <behaviortree_cpp_v3/decorator_node.h>
#include <fkie_behavior_tree_manager/logging.h>

#include <topic_tools/shape_shifter.h>

namespace BehaviorTreeNodes
{
/**
 * @brief Asynchronously wait until a generic message is received, return RUNNING while waiting
 * ROS Params:
 *        WaitForGenericMsgAsyncAction/[name]_topic_in: Input topic for generic message (any)
 *        WaitForGenericMsgAsyncAction/[name]_timeout:  Timeout to return FAILURE, (-1.0 disable)
 */
class WaitForGenericMsgAsyncAction : public BT::CoroActionNode
{
public:
  std::string name;
  ros::Subscriber sub;
  std::string topic_in;
  float timeout = -1.0f;
  bool valid_init = false;
  bool message_received = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports;
    return ports;
  }

  WaitForGenericMsgAsyncAction(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::CoroActionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;

    std::string parameter_topic_in = "WaitForGenericMsgAsyncAction/" + _name + "_topic_in";
    std::string parameter_timeout = "WaitForGenericMsgAsyncAction/" + _name + "_timeout";

    private_node.param<std::string>(parameter_topic_in, topic_in, topic_in);
    private_node.param<float>(parameter_timeout, timeout, timeout);

    if (topic_in.empty())
    {
      BT_ROS_ERROR_STREAM(name << " (in) topic is empty. Check parameter: [" << topic_in << "]");
      return;
    }

    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
    callback = [&](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { topicCallback(msg, topic_in); };
    sub = public_node.subscribe(topic_in, 10, callback);

    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    message_received = false;

    ros::spinOnce();

    ros::Time begin = ros::Time::now();
    while (ros::ok() && !message_received)
    {
      ros::spinOnce();
      ros::Duration(0.05).sleep();

      // timeout was defined
      if (timeout > 0)
      {
        // waiting for message or timeout
        ros::Duration waiting_time = ros::Time::now() - begin;
        if (waiting_time.toSec() >= timeout)
        {
          BT_ROS_WARN_STREAM(name << "Timeout reached, assume the action has failed!");
          return BT::NodeStatus::FAILURE;
        }
      }

      // set status to RUNNING and "pause/sleep"
      // If halt() is called, we will not resume execution (stack destroyed)
      setStatusRunningAndYield();
    }

    message_received = false;
    return BT::NodeStatus::SUCCESS;
  }

  void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name)
  {
    message_received = true;
  }

  void halt() override
  {
    BT_ROS_DEBUG_STREAM(name << "node has been halted!");
    message_received = true;
    setStatus(BT::NodeStatus::FAILURE);

    CoroActionNode::halt();
  }
};

}  // namespace BehaviorTreeNodes

#endif  // WAIT_FOR_GENERIC_MSG_ASYNC_ACTION_NODE_H