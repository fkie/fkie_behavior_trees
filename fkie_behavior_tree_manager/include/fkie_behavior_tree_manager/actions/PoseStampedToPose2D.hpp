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

#ifndef POSE_3D_TO_POSE_2D_NODE_H
#define POSE_3D_TO_POSE_2D_NODE_H

#include <behaviortree_cpp_v3/decorator_node.h>
#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

namespace BehaviorTreeNodes
{
class PoseStampedToPose2D : public BT::SyncActionNode
{
public:
  std::string name;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<geometry_msgs::PoseStamped>("target_pose_3d"), BT::OutputPort<Pose2D>("target"
                                                                                                                "_pose_"
                                                                                                                "2d") };
    return ports;
  }

  PoseStampedToPose2D(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(_name, config), name("[" + _name + "] ")
  {
    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    geometry_msgs::PoseStamped target_pose_3d;
    auto res = BT::TreeNode::getInput("target_pose_3d", target_pose_3d);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << " Could not load target_pose_3d, resetting variable. Error: " << res.error());
      BT::TreeNode::setOutput("target_pose_3d", geometry_msgs::PoseStamped());
      return BT::NodeStatus::FAILURE;
    }

    Pose2D target_pose_2d(target_pose_3d);
    BT_ROS_DEBUG_STREAM(name << " target_pose_2d: " << target_pose_2d.toString());

    // save current pose to blackboard variable
    res = BT::TreeNode::setOutput("target_pose_2d", target_pose_2d);
    if (!res)
    {
      BT_ROS_ERROR_STREAM(name << " Could not set output target_pose_2d: " << res.error());
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      return BT::NodeStatus::SUCCESS;
    }
  }
};

}  // namespace BehaviorTreeNodes
#endif  // POSE_3D_TO_POSE_2D_NODE_H