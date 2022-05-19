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

#ifndef POSE_STAMPED_RECEIVED_NODE_H
#define POSE_STAMPED_RECEIVED_NODE_H

#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <fkie_behavior_tree_manager/logging.h>

namespace BehaviorTreeNodes
{
class PoseStampedReceived : public BT::ConditionNode
{
public:
  std::string name;
  ros::Subscriber sub_pose;
  std::string topic_pose;

  geometry_msgs::PoseStamped current_pose;
  bool new_pose_arrived = false;

  bool valid_init = false;

  PoseStampedReceived(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::OutputPort<geometry_msgs::PoseStamped>("target_pose_3d") };
    return ports;
  }

  BT::NodeStatus tick() override;
  void callbackPose(const geometry_msgs::PoseStamped& msg);
};

inline BT::NodeStatus PoseStampedReceived::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!valid_init)
    return BT::NodeStatus::FAILURE;

  if (new_pose_arrived)
  {
    auto res = BT::TreeNode::setOutput("target_pose_3d", current_pose);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << "Could not set output target_pose: " << res.error());
      return BT::NodeStatus::FAILURE;
    }

    new_pose_arrived = false;

    std::cout << "New pose_stamped was received." << std::endl;
    BT_ROS_DEBUG_STREAM(name << "New goal pose was received.");
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}

PoseStampedReceived::PoseStampedReceived(const std::string& _name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(_name, config), name("[" + _name + "] ")
{
  ros::NodeHandle private_node("~");
  ros::NodeHandle public_node;

  std::string parameter_topic_pose = "PoseStampedReceived/" + _name + "_topic_pose";

  private_node.param<std::string>(parameter_topic_pose, topic_pose, "pose_in");

  if (topic_pose.empty())
  {
    BT_ROS_ERROR_STREAM(name << " [topic_pose] is empty, check parameter [" << parameter_topic_pose << "]");
    return;
  }

  sub_pose = public_node.subscribe(topic_pose, 1, &PoseStampedReceived::callbackPose, this);
  valid_init = true;

  BT_ROS_INFO_STREAM(name << " initialized");
}

void PoseStampedReceived::callbackPose(const geometry_msgs::PoseStamped& msg)
{
  if (msg.header.frame_id.empty())
  {
    BT_ROS_WARN_STREAM(name << "ignoring message with empty frame_id");
    return;
  }

  if (std::isnan(msg.pose.position.x) || std::isnan(msg.pose.position.y) || std::isnan(msg.pose.position.z))
  {
    BT_ROS_WARN_STREAM(name << "invalid pose received, ignoring.");
    return;
  }

  current_pose = msg;
  new_pose_arrived = true;
  BT_ROS_DEBUG_STREAM(name << " New message received with frame [" << msg.header.frame_id << "]");
}

}  // namespace BehaviorTreeNodes

#endif  // POSE_STAMPED_RECEIVED_NODE_H
