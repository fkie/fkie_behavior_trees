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

#ifndef RELATIVE_3D_POSE_ACTION_H
#define RELATIVE_3D_POSE_ACTION_H

#include <behaviortree_cpp_v3/decorator_node.h>
#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace BehaviorTreeNodes
{
class Relative3DPoseAction : public BT::SyncActionNode
{
public:
  std::string name;
  ros::Publisher pub_3d;
  std::string relative_frame;

  std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
  tf2_ros::Buffer tf2_buffer;

public:
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::BidirectionalPort<geometry_msgs::PoseStamped>("target_pose_3d") };
    return ports;
  }

  Relative3DPoseAction(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle public_node;
    ros::NodeHandle private_node("~");

    // Publish relative 3d pose
    pub_3d = public_node.advertise<geometry_msgs::PoseStamped>("relative_pose_3d", 10, false);

    // Get the relative frame
    std::string parameter_relative_frame = "Relative3DPoseAction/" + _name + "_relative_frame";
    private_node.param<std::string>(parameter_relative_frame, relative_frame, std::string(""));

    tf2_listener = std::make_unique<tf2_ros::TransformListener>(tf2_buffer);

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    // Validate the relative frame
    if (relative_frame.empty())
    {
      BT_ROS_ERROR_STREAM(name << " relative_frame is empty");
      return BT::NodeStatus::FAILURE;
    }

    // Validate 3D goal
    geometry_msgs::PoseStamped target_pose_3d;
    auto res = BT::TreeNode::getInput("target_pose_3d", target_pose_3d);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << " Could not load target_pose_3d, resetting variable. Error: " << res.error());
      BT::TreeNode::setOutput("target_pose_3d", geometry_msgs::PoseStamped());
      return BT::NodeStatus::FAILURE;
    }

    target_pose_3d.header.stamp = ros::Time::now();

    // compute relative pose
    geometry_msgs::PoseStamped relative_pose;
    tf2_buffer.transform(target_pose_3d, relative_pose, relative_frame, ros::Duration(0.5));

    BT_ROS_WARN_STREAM(name << " Frame id for relative pose: " << relative_pose.header.frame_id);
    BT_ROS_WARN_STREAM(name << " Received GOAL 3D pose: " << target_pose_3d.pose.position.x << ", "
                            << target_pose_3d.pose.position.y << ", " << target_pose_3d.pose.position.z);
    BT_ROS_WARN_STREAM(name << " Transformed Relative 3D pose: " << relative_pose.pose.position.x << ", "
                            << relative_pose.pose.position.y << ", " << relative_pose.pose.position.z);

    pub_3d.publish(relative_pose);

    // save relative_pose to blackboard variable
    res = BT::TreeNode::setOutput("target_pose_3d", relative_pose);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << " Could not set output target_pose_2d: " << res.error());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace BehaviorTreeNodes
#endif  // RELATIVE_3D_POSE_ACTION_H