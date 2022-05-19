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

#include "NotifyNavigationResult.h"

namespace MoveBaseActionsGroup
{
NotifyNavigationResult::NotifyNavigationResult(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  ros::NodeHandle public_node("");
  ros::NodeHandle private_node("~");
  private_node.param("MoveBaseActions/topic_notify_result", topic_notify_result, std::string("navigation_result"));

  if (topic_notify_result.empty())
  {
    BT_ROS_ERROR("Missing param [MoveBaseActions/topic_notify_result]");
    return;
  }

  pub_notify = public_node.advertise<fkie_bt_move_base_actions::NavigationResult>(topic_notify_result, 10, false);

  is_initialized = true;
  BT_ROS_INFO_STREAM("Successfully initialized");
}

NotifyNavigationResult::~NotifyNavigationResult()
{
}

BT::NodeStatus NotifyNavigationResult::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!is_initialized)
  {
    BT_ROS_ERROR("NotifyNavigationResult is not initialized");
    return BT::NodeStatus::FAILURE;
  }

  unsigned int nav_error_code = -1;
  auto res = BT::TreeNode::getInput("nav_error_code", nav_error_code);
  if (!res)
  {
    BT::TreeNode::setOutput("nav_error_code", nav_error_code);
    return BT::NodeStatus::FAILURE;
  }

  BT_ROS_DEBUG_STREAM("nav_error_code: " << nav_error_code);

  fkie_bt_move_base_actions::NavigationResult nr;
  nr.header.stamp = ros::Time::now();
  nr.nav_error_code = nav_error_code;
  updateErrorDescriptionAndOrigin(nr);
  pub_notify.publish(nr);

  return (nav_error_code == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void NotifyNavigationResult::updateErrorDescriptionAndOrigin(fkie_bt_move_base_actions::NavigationResult& nr) const
{
  // clang-format off
  switch (nr.nav_error_code)
  {
    case 0: set(nr, "move_base_flex", "SUCCESS"); break;

    case 10: set(nr, "move_base_interface", "FAILURE"); break;
    case 11: set(nr, "move_base_interface", "CANCELED"); break;
    case 12: set(nr, "move_base_interface", "COLLISION"); break;
    case 13: set(nr, "move_base_interface", "OSCILLATION"); break;
    case 14: set(nr, "move_base_interface", "START_BLOCKED"); break;
    case 15: set(nr, "move_base_interface", "GOAL_BLOCKED"); break;
    case 16: set(nr, "move_base_interface", "TF_ERROR"); break;
    case 17: set(nr, "move_base_interface", "INTERNAL_ERROR"); break;

    case 50: set(nr, "global_planner", "FAILURE"); break;
    case 51: set(nr, "global_planner", "CANCELED"); break;
    case 52: set(nr, "global_planner", "INVALID_START"); break;
    case 53: set(nr, "global_planner", "INVALID_GOAL"); break;
    case 54: set(nr, "global_planner", "NO_PATH_FOUND"); break;
    case 55: set(nr, "global_planner", "PAT_EXCEEDED"); break;
    case 56: set(nr, "global_planner", "EMPTY_PATH"); break;
    case 57: set(nr, "global_planner", "TF_ERROR"); break;
    case 58: set(nr, "global_planner", "NOT_INITIALIZED"); break;
    case 59: set(nr, "global_planner", "INVALID_PLUGIN"); break;
    case 60: set(nr, "global_planner", "INTERNAL_ERROR"); break;
    case 61: set(nr, "global_planner", "OUT_OF_MAP"); break;
    case 62: set(nr, "global_planner", "MAP_ERROR"); break;
    case 63: set(nr, "global_planner", "STOPPED"); break;

    case 100: set(nr, "local_navigation", "FAILURE"); break;
    case 101: set(nr, "local_navigation", "CANCELED"); break;
    case 102: set(nr, "local_navigation", "NO_VALID_CMD"); break;
    case 103: set(nr, "local_navigation", "PAT_EXCEEDED"); break;
    case 104: set(nr, "local_navigation", "COLLISION"); break;
    case 105: set(nr, "local_navigation", "OSCILLATION"); break;
    case 106: set(nr, "local_navigation", "ROBOT_STUCK"); break;
    case 107: set(nr, "local_navigation", "MISSED_GOAL"); break;
    case 108: set(nr, "local_navigation", "MISSED_PATH"); break;
    case 109: set(nr, "local_navigation", "BLOCKED_PATH"); break;
    case 110: set(nr, "local_navigation", "INVALID_PATH"); break;
    case 111: set(nr, "local_navigation", "TF_ERROR"); break;
    case 112: set(nr, "local_navigation", "NOT_INITIALIZED"); break;
    case 113: set(nr, "local_navigation", "INVALID_PLUGIN"); break;
    case 114: set(nr, "local_navigation", "INTERNAL_ERROR"); break;
    case 115: set(nr, "local_navigation", "OUT_OF_MAP"); break;
    case 116: set(nr, "local_navigation", "MAP_ERROR"); break;
    case 117: set(nr, "local_navigation", "STOPPED"); break;

    case 150: set(nr, "recovery", "FAILURE"); break;
    case 151: set(nr, "recovery", "CANCELED"); break;
    case 152: set(nr, "recovery", "PAT_EXCEEDED"); break;
    case 153: set(nr, "recovery", "TF_ERROR"); break;
    case 154: set(nr, "recovery", "NOT_INITIALIZED"); break;
    case 155: set(nr, "recovery", "INVALID_PLUGIN"); break;
    case 156: set(nr, "recovery", "INTERNAL_ERROR"); break;
    case 157: set(nr, "recovery", "STOPPED"); break;
    case 158: set(nr, "recovery", "IMPASSABLE"); break;

    default: set(nr, "unknown", "unknown error code"); break;
  }
  // clang-format on
}

void NotifyNavigationResult::set(fkie_bt_move_base_actions::NavigationResult& nr, std::string origin,
                                 std::string description) const
{
  nr.origin = origin;
  nr.description = description;
}

}  // namespace MoveBaseActionsGroup
