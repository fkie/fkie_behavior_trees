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

#ifndef BEHAVIOR_TREE_NODES_H
#define BEHAVIOR_TREE_NODES_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

#include <fkie_behavior_tree_manager/actions/BoolAction.hpp>
#include <fkie_behavior_tree_manager/actions/DelayAction.hpp>
#include <fkie_behavior_tree_manager/actions/PoseStampedToPose2D.hpp>
#include <fkie_behavior_tree_manager/actions/PublishPathAction.hpp>
#include <fkie_behavior_tree_manager/actions/PublishPoseAction.hpp>
#include <fkie_behavior_tree_manager/actions/PublishPoseStampedAction.hpp>
#include <fkie_behavior_tree_manager/actions/Relative3DPoseAction.hpp>
#include <fkie_behavior_tree_manager/actions/ServiceEmptyAction.hpp>
#include <fkie_behavior_tree_manager/actions/SubscribePoseAction.hpp>
#include <fkie_behavior_tree_manager/actions/SubscribePoseStampedAction.hpp>
#include <fkie_behavior_tree_manager/actions/WaitForGenericMsgAsyncAction.hpp>

#include <fkie_behavior_tree_manager/conditions/BoolCondition.hpp>
#include <fkie_behavior_tree_manager/conditions/CheckPoseReachedAsync.hpp>
#include <fkie_behavior_tree_manager/conditions/CheckTimeoutAsync.hpp>
#include <fkie_behavior_tree_manager/conditions/GenericMsgReceived.hpp>
#include <fkie_behavior_tree_manager/conditions/PoseReached.hpp>
#include <fkie_behavior_tree_manager/conditions/PoseReceived.hpp>
#include <fkie_behavior_tree_manager/conditions/PoseStampedReceived.hpp>
#include <fkie_behavior_tree_manager/conditions/TFTransformAsyncCondition.hpp>
#include <fkie_behavior_tree_manager/conditions/TFTransformCondition.hpp>
#include <fkie_behavior_tree_manager/conditions/ValidPose.hpp>
#include <fkie_behavior_tree_manager/conditions/checkBooleanPort.hpp>
#include <fkie_behavior_tree_manager/conditions/checkCommand.hpp>

#include <fkie_behavior_tree_manager/decorators/CommandSwitch.hpp>
#include <fkie_behavior_tree_manager/decorators/CommandTrigger.hpp>
#include <fkie_behavior_tree_manager/decorators/ConditionalLoop.hpp>
#include <fkie_behavior_tree_manager/decorators/RateController.hpp>
#include <fkie_behavior_tree_manager/decorators/RunOnlyOnce.hpp>

namespace BehaviorTreeNodes
{
inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  // register actions
  factory.registerNodeType<BoolAction>("BoolAction");
  factory.registerNodeType<DelayAction>("DelayAction");
  factory.registerNodeType<ServiceEmptyAction>("ServiceEmptyAction");
  factory.registerNodeType<PublishPoseAction>("PublishPoseAction");
  factory.registerNodeType<PublishPathAction>("PublishPathAction");
  factory.registerNodeType<PublishPoseStampedAction>("PublishPoseStampedAction");
  factory.registerNodeType<SubscribePoseAction>("SubscribePoseAction");
  factory.registerNodeType<SubscribePoseStampedAction>("SubscribePoseStampedAction");
  factory.registerNodeType<PoseStampedToPose2D>("PoseStampedToPose2D");
  factory.registerNodeType<Relative3DPoseAction>("Relative3DPoseAction");
  factory.registerNodeType<WaitForGenericMsgAsyncAction>("WaitForGenericMsgAsyncAction");

  // register conditions
  BT::PortsList checkCommandPorts = { BT::InputPort<std::string>("cmd") };
  factory.registerSimpleCondition("checkCommand", checkCommand, checkCommandPorts);

  BT::PortsList checkBooleanPortPorts = { BT::InputPort<bool>("port_value"), BT::InputPort<bool>("success_value") };
  factory.registerSimpleCondition("CheckBooleanPort", CheckBooleanPort, checkBooleanPortPorts);

  BT::PortsList ValidPosePorts = { BT::BidirectionalPort<Pose2D>("target_pose") };
  factory.registerSimpleCondition("ValidPose", ValidPose, ValidPosePorts);

  factory.registerNodeType<PoseReceived>("PoseReceived");
  factory.registerNodeType<PoseStampedReceived>("PoseStampedReceived");
  factory.registerNodeType<PoseReached>("PoseReached");
  factory.registerNodeType<CheckPoseReachedAsync>("CheckPoseReachedAsync");
  factory.registerNodeType<CheckTimeoutAsync>("CheckTimeoutAsync");
  factory.registerNodeType<BoolCondition>("BoolCondition");
  factory.registerNodeType<GenericMsgReceived>("GenericMsgReceived");
  factory.registerNodeType<TFTransformCondition>("TFTransformCondition");
  factory.registerNodeType<TFTransformAsyncCondition>("TFTransformAsyncCondition");

  // register decorators
  factory.registerNodeType<RateController>("RateController");
  factory.registerNodeType<CommandSwitch>("CommandSwitch");
  factory.registerNodeType<CommandTrigger>("CommandTrigger");
  factory.registerNodeType<RunOnlyOnce>("RunOnlyOnce");
  factory.registerNodeType<ConditionalLoop>("ConditionalLoop");
}

}  // namespace BehaviorTreeNodes

#endif  // BEHAVIOR_TREE_NODES_H