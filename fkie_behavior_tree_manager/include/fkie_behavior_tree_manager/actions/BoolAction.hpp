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

#ifndef BOOL_ACTION_NODE_H
#define BOOL_ACTION_NODE_H

#include <behaviortree_cpp_v3/decorator_node.h>
#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <std_msgs/Bool.h>
#include <string>

namespace BehaviorTreeNodes
{
class BoolAction : public BT::ActionNodeBase
{
public:
  std::string name;
  ros::Subscriber sub_bool;
  ros::Publisher pub_sub;
  std::string topic_in, topic_out;
  float timeout;
  bool valid_init = false;
  bool message_received = false;
  bool result_action = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports;
    return ports;
  }

  BoolAction(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;

    std::string parameter_topic_in = "BoolAction/" + _name + "_topic_in";
    std::string parameter_topic_out = "BoolAction/" + _name + "_topic_out";
    std::string parameter_timeout = "BoolAction/" + _name + "_timeout";

    private_node.param<std::string>(parameter_topic_in, topic_in, std::string(""));
    private_node.param<std::string>(parameter_topic_out, topic_out, std::string(""));
    private_node.param<float>(parameter_timeout, timeout, -1.0f);

    if (topic_in.empty() || topic_out.empty())
    {
      BT_ROS_ERROR_STREAM(name << " (in/out) topic are empty. Check parameters: [" << topic_in << "] and [" << topic_out
                               << "]");
      return;
    }

    pub_sub = public_node.advertise<std_msgs::Bool>(topic_out, 10, false);
    sub_bool = public_node.subscribe(topic_in, 5, &BoolAction::callbackBoolAction, this);
    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    message_received = false;
    result_action = false;

    std_msgs::Bool msg;
    msg.data = true;
    pub_sub.publish(msg);
    ros::spinOnce();

    ros::Time begin = ros::Time::now();
    while (ros::ok() && !message_received)
    {
      ros::spinOnce();

      // timeout was defined
      if (timeout > 0)
      {
        // waiting for message or timeout
        ros::Duration waiting_time = ros::Time::now() - begin;
        if (waiting_time.toSec() >= timeout)
        {
          BT_ROS_WARN_STREAM("BoolAction: Timeout reached, assume the action has failed!");
          result_action = false;
          break;
        }
      }

      ros::Duration(0.05).sleep();
    }

    if (result_action)
      return BT::NodeStatus::SUCCESS;
    else
      return BT::NodeStatus::FAILURE;
  }

  void callbackBoolAction(const std_msgs::Bool& msg)
  {
    message_received = true;
    result_action = msg.data;
  }

  void halt() override
  {
    message_received = true;
    result_action = false;
    setStatus(BT::NodeStatus::IDLE);
  }
};

}  // namespace BehaviorTreeNodes

#endif  // BOOL_ACTION_NODE_H