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

#ifndef VALID_POSE_H
#define VALID_POSE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

namespace BehaviorTreeNodes
{
BT::NodeStatus ValidPose(BT::TreeNode& self)
{
  Pose2D target_pose;
  auto res = self.getInput("target_pose", target_pose);
  if (!res)
  {
    self.setOutput("target_pose", Pose2D());
    return BT::NodeStatus::FAILURE;
  }

  if (target_pose.frame_id.empty())
  {
    BT_ROS_WARN_STREAM(self.name() << ": Frame_id is empty, returning Failure");
    return BT::NodeStatus::FAILURE;
  }

  if (!std::isfinite(target_pose.x) || !std::isfinite(target_pose.y))
  {
    BT_ROS_WARN_STREAM(self.name() << ": invalid (x,y):" << target_pose.toString());
    return BT::NodeStatus::FAILURE;
  }

  BT_ROS_DEBUG_STREAM(self.name() << ": pose is valid");
  return BT::NodeStatus::SUCCESS;
};

}  // namespace BehaviorTreeNodes

#endif  // VALID_POSE_H