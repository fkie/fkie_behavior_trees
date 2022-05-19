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

#ifndef MOVEIT_ACTIONS_H
#define MOVEIT_ACTIONS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "fkie_bt_moveit_actions/actions/MoveitCartesianPathPlanning.hpp"
#include "fkie_bt_moveit_actions/actions/MoveitExecuteWaypoints.hpp"
#include "fkie_bt_moveit_actions/actions/MoveitMoveToFixConfiguration.hpp"
#include "fkie_bt_moveit_actions/actions/MoveitMoveToPose.hpp"
#include "fkie_bt_moveit_actions/actions/MoveitPlanToPose.hpp"
#include "fkie_bt_moveit_actions/actions/MoveitExecutePlan.hpp"
#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <ros/ros.h>

namespace MoveitActionsGroup
{
inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  // moveit actions
  factory.registerNodeType<MoveitCartesianPathPlanning>("MoveitCartesianPathPlanning");
  factory.registerNodeType<MoveitExecuteWaypoints>("MoveitExecuteWaypoints");
  factory.registerNodeType<MoveitMoveToPose>("MoveitMoveToPose");
  factory.registerNodeType<MoveitPlanToPose>("MoveitPlanToPose");
  factory.registerNodeType<MoveitExecutePlan>("MoveitExecutePlan");
  factory.registerNodeType<MoveitMoveToFixConfiguration>("MoveitMoveToFixConfiguration");
}
}  // namespace MoveitActionsGroup

#endif  // MOVEIT_ACTIONS_H
