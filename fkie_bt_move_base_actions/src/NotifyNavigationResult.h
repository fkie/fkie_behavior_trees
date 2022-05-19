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

#ifndef MOVE_BASE_NOTIFY_NAVIGATION_RESULT_H
#define MOVE_BASE_NOTIFY_NAVIGATION_RESULT_H

#include <ros/ros.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <fkie_bt_move_base_actions/NavigationResult.h>

namespace MoveBaseActionsGroup
{
/* Error codes defined by Move Base Flex messages
    uint8 SUCCESS         = 0

  Global path planning errors:
    uint8 FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
    uint8 CANCELED        = 51  # The action has been canceled by a action client
    uint8 INVALID_START   = 52  #
    uint8 INVALID_GOAL    = 53
    uint8 NO_PATH_FOUND   = 54
    uint8 PAT_EXCEEDED    = 55
    uint8 EMPTY_PATH      = 56
    uint8 TF_ERROR        = 57
    uint8 NOT_INITIALIZED = 58
    uint8 INVALID_PLUGIN  = 59
    uint8 INTERNAL_ERROR  = 60
    uint8 OUT_OF_MAP      = 61
    uint8 MAP_ERROR       = 62
    uint8 STOPPED         = 63  # The planner execution has been stopped rigorously.
    # 71..99 are reserved as plugin specific errors

  Local navigation errors:
    uint8 FAILURE         = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
    uint8 CANCELED        = 101
    uint8 NO_VALID_CMD    = 102
    uint8 PAT_EXCEEDED    = 103
    uint8 COLLISION       = 104
    uint8 OSCILLATION     = 105
    uint8 ROBOT_STUCK     = 106
    uint8 MISSED_GOAL     = 107
    uint8 MISSED_PATH     = 108
    uint8 BLOCKED_PATH    = 109
    uint8 INVALID_PATH    = 110
    uint8 TF_ERROR        = 111
    uint8 NOT_INITIALIZED = 112
    uint8 INVALID_PLUGIN  = 113
    uint8 INTERNAL_ERROR  = 114
    uint8 OUT_OF_MAP      = 115  # The start and / or the goal are outside the map
    uint8 MAP_ERROR       = 116  # The map is not running properly
    uint8 STOPPED         = 117  # The controller execution has been stopped rigorously.
    # 121..149 are reserved as plugin specific errors

  Recovery errors:
    uint8 FAILURE         = 150
    uint8 CANCELED        = 151
    uint8 PAT_EXCEEDED    = 152
    uint8 TF_ERROR        = 153
    uint8 NOT_INITIALIZED = 154
    uint8 INVALID_PLUGIN  = 155
    uint8 INTERNAL_ERROR  = 156
    uint8 STOPPED         = 157  # The recovery behaviour execution has been stopped rigorously.
    uint8 IMPASSABLE      = 158  # Further execution would lead to a collision
    # 171..199 are reserved as plugin specific errors

  Move base interface errors:
    uint8 FAILURE        = 10
    uint8 CANCELED       = 11
    uint8 COLLISION      = 12
    uint8 OSCILLATION    = 13
    uint8 START_BLOCKED  = 14
    uint8 GOAL_BLOCKED   = 15
    uint8 TF_ERROR       = 16
    uint8 INTERNAL_ERROR = 17
    # 21..49 are reserved for future general error codes
  */
class NotifyNavigationResult : public BT::SyncActionNode
{
public:
  bool is_initialized = false;
  std::string topic_notify_result;

  NotifyNavigationResult(const std::string& name, const BT::NodeConfiguration& config);
  ~NotifyNavigationResult();
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::BidirectionalPort<unsigned int>("nav_error_code") };
    return ports;
  }

  void updateErrorDescriptionAndOrigin(fkie_bt_move_base_actions::NavigationResult& nr) const;
  void set(fkie_bt_move_base_actions::NavigationResult& nr, std::string origin, std::string description) const;

private:
  ros::Publisher pub_notify;
};
}  // namespace MoveBaseActionsGroup
#endif  // MOVE_BASE_NOTIFY_NAVIGATION_RESULT_H
