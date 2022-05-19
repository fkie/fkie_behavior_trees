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

#ifndef PUBLISH_PATH_ACTION_NODE_H
#define PUBLISH_PATH_ACTION_NODE_H

#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <string>

#include <behaviortree_cpp_v3/decorator_node.h>
#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

namespace BehaviorTreeNodes
{
class PublishPathAction : public BT::SyncActionNode
{
public:
  std::string name;
  ros::NodeHandle public_node;
  std::string topic_path_out;
  ros::Publisher pub_path;

  bool valid_init = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::BidirectionalPort<nav_msgs::Path>("path") };
    return ports;
  }

  PublishPathAction(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");

    std::string parameter_topic_path_out = "PublishPathAction/" + _name + "_topic_path_out";
    private_node.param<std::string>(parameter_topic_path_out, topic_path_out, std::string(""));

    if (topic_path_out.empty())
    {
      BT_ROS_ERROR_STREAM(name << " topic_path_out is empty. Check parameter [" << parameter_topic_path_out << "]");
      return;
    }

    pub_path = public_node.advertise<nav_msgs::Path>(topic_path_out, 10);
    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    // get target pose
    nav_msgs::Path path;
    auto res = BT::TreeNode::getInput("path", path);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << " Could not load path, resetting variable. Error: " << res.error());
      BT::TreeNode::setOutput("path", nav_msgs::Path());  // reset current path
      return BT::NodeStatus::FAILURE;
    }

    if (path.poses.size() == 0)
    {
      BT_ROS_ERROR_STREAM(name << "Invalid path with 0 poses. ");
      return BT::NodeStatus::FAILURE;
    }

    if (path.header.frame_id.empty())
    {
      BT_ROS_ERROR_STREAM(name << "Empty frame id.");
      return BT::NodeStatus::FAILURE;
    }

    pub_path.publish(path);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace BehaviorTreeNodes

#endif  // PUBLISH_PATH_ACTION_NODE_H