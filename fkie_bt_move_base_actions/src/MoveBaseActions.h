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

#ifndef MOVE_BASE_ACTIONS_H
#define MOVE_BASE_ACTIONS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "CancelExePathSync.h"
#include "ExePathAsync.h"
#include "ExePathSync.h"
#include "GetPath.h"
#include "MoveBaseUtils.h"
#include "NavigateToGoal.h"
#include "NavigateToGoalSync.h"
#include "NotifyNavigationResult.h"
#include "Recovery.h"

namespace MoveBaseActionsGroup
{
inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  BT::PortsList newTargetPoseReceivedPorts = { BT::OutputPort<Pose2D>("target_pose") };
  factory.registerSimpleCondition("MoveBase_newTargetPoseReceived", newTargetPoseReceived, newTargetPoseReceivedPorts);

  BT::PortsList hasValidGoalPorts = { BT::BidirectionalPort<Pose2D>("target_pose") };
  factory.registerSimpleCondition("MoveBase_hasValidGoal", hasValidGoal, hasValidGoalPorts);

  BT::PortsList CancelCurrentPoseActionPorts = { BT::OutputPort<Pose2D>("target_pose") };
  factory.registerSimpleAction("CancelCurrentPoseAction", CancelCurrentPoseAction, CancelCurrentPoseActionPorts);

  // BT::PortsList moveBaseHeartbeatPorts = {BT::InputPort<geometry_msgs::PoseStamped>("message")};
  // factory.registerSimpleAction("MoveBase_heartbeat", moveBaseHeartbeat, moveBaseHeartbeatPorts);

  factory.registerNodeType<ExePathSync>("MoveBase_ExePathSync");
  factory.registerNodeType<ExePathAsync>("MoveBase_ExePathAsync");
  factory.registerNodeType<CancelExePathSync>("CancelExePathSync");
  factory.registerNodeType<GetPath>("MoveBase_GetPath");
  factory.registerNodeType<Recovery>("MoveBase_Recovery");
  factory.registerNodeType<NotifyNavigationResult>("MoveBase_NotifyNavigationResult");
  factory.registerNodeType<NavigateToGoal>("MoveBase_NavigateToGoal");
  factory.registerNodeType<NavigateToGoalSync>("MoveBase_NavigateToGoalSync");
}

}  // namespace MoveBaseActionsGroup

#endif  // MOVE_BASE_ACTIONS_H