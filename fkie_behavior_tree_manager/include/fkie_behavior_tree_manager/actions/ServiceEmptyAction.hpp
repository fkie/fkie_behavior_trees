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

#ifndef SERVICE_EMPTY_NODE_H
#define SERVICE_EMPTY_NODE_H

#include <behaviortree_cpp_v3/decorator_node.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <std_srvs/Empty.h>
#include <string>

namespace BehaviorTreeNodes
{
class ServiceEmptyAction : public BT::SyncActionNode
{
public:
  std::string name;
  ros::NodeHandle public_node;
  std::string topic_service;
  double delay = 0.0;
  bool valid_init = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports;
    return ports;
  }

  ServiceEmptyAction(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");

    std::string parameter_topic_service = "ServiceEmptyAction/" + _name + "_topic_service";
    std::string parameter_delay = "ServiceEmptyAction/" + _name + "_delay";

    private_node.param<std::string>(parameter_topic_service, topic_service, std::string(""));
    private_node.param<double>(parameter_delay, delay, 0.0);

    if (topic_service.empty())
    {
      BT_ROS_ERROR_STREAM(name << " topic_service is empty. Check parameter [" << parameter_topic_service << "]");
      return;
    }

    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    BT_ROS_DEBUG_STREAM(name << " Calling service: " << topic_service);
    ros::ServiceClient client = public_node.serviceClient<std_srvs::Empty>(topic_service);
    std_srvs::Empty srv;
    if (client.call(srv))
    {
      BT_ROS_DEBUG_STREAM(name << " Service executed successfully");
      ros::Duration(delay).sleep();
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      BT_ROS_ERROR_STREAM(name << " Service call to Octomap failed. Is Octomap server running?");
      return BT::NodeStatus::FAILURE;
    }
  }
};

}  // namespace BehaviorTreeNodes

#endif  // SERVICE_EMPTY_NODE_H
