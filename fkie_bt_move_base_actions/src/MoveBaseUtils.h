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

#ifndef MOVE_BASE_SIMPLE_ACTIONS_H
#define MOVE_BASE_SIMPLE_ACTIONS_H

#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/RecoveryAction.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>
#include <tf/transform_listener.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> ClientUtilsExePathAction;

namespace MoveBaseActionsGroup
{
BT::NodeStatus moveBaseHeartbeat(BT::TreeNode& self);

BT::NodeStatus newTargetPoseReceived(BT::TreeNode& self);

BT::NodeStatus CancelCurrentPoseAction(BT::TreeNode& self);

BT::NodeStatus hasValidGoal(BT::TreeNode& self);

}  // namespace MoveBaseActionsGroup

#endif  // MOVE_BASE_SIMPLE_ACTIONS_H