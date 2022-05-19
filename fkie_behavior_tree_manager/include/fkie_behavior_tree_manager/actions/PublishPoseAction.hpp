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

#ifndef PUBLISH_POSE_ACTION_NODE_H
#define PUBLISH_POSE_ACTION_NODE_H

#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <string>

#include <behaviortree_cpp_v3/decorator_node.h>
#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

namespace BehaviorTreeNodes
{
class PublishPoseAction : public BT::SyncActionNode
{
public:
  std::string name;
  ros::NodeHandle public_node;
  std::string topic_pose_out;
  ros::Publisher pub_pose;

  bool valid_init = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::BidirectionalPort<Pose2D>("target_pose") };
    return ports;
  }

  PublishPoseAction(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");

    std::string parameter_topic_pose_out = "PublishPoseAction/" + _name + "_topic_pose_out";
    private_node.param<std::string>(parameter_topic_pose_out, topic_pose_out, std::string(""));

    if (topic_pose_out.empty())
    {
      BT_ROS_ERROR_STREAM(name << " topic_pose_out is empty. Check parameter [" << parameter_topic_pose_out << "]");
      return;
    }

    pub_pose = public_node.advertise<geometry_msgs::PoseStamped>(topic_pose_out, 10);
    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    // get target pose
    Pose2D target_pose;
    auto res = BT::TreeNode::getInput("target_pose", target_pose);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << " Could not load target_pose, resetting variable. Error: " << res.error());
      BT::TreeNode::setOutput("target_pose", Pose2D());  // reset current target_pose
      return BT::NodeStatus::FAILURE;
    }

    if (std::isnan(target_pose.x) || std::isnan(target_pose.y) || std::isnan(target_pose.theta))
    {
      BT_ROS_ERROR_STREAM(name << "Invalid pose: " << target_pose.toString());
      return BT::NodeStatus::FAILURE;
    }

    if (target_pose.frame_id.empty())
    {
      BT_ROS_ERROR_STREAM(name << "Empty frame id: " << target_pose.toString());
      return BT::NodeStatus::FAILURE;
    }

    // publish target pose
    geometry_msgs::PoseStamped pose;
    pose = target_pose.toPoseStamped();
    pub_pose.publish(pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace BehaviorTreeNodes

#endif  // PUBLISH_POSE_ACTION_NODE_H