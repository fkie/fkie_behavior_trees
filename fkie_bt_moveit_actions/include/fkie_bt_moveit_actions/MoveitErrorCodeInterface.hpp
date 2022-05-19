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

#ifndef MOVEIT_ERROR_CODE_INTERFACE_H
#define MOVEIT_ERROR_CODE_INTERFACE_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>

inline std::string getMoveitErrorCodeDescription(moveit::core::MoveItErrorCode ec)
{
  switch (ec.val)
  {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
      return std::string("SUCCESS");
    case moveit_msgs::MoveItErrorCodes::FAILURE:
      return std::string("FAILURE");
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
      return std::string("PLANNING_FAILED");
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
      return std::string("INVALID_MOTION_PLAN");
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return std::string("MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE");
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
      return std::string("CONTROL_FAILED");
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
      return std::string("UNABLE_TO_AQUIRE_SENSOR_DATA");
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
      return std::string("TIMED_OUT");
    case moveit_msgs::MoveItErrorCodes::PREEMPTED:
      return std::string("PREEMPTED");
    case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
      return std::string("START_STATE_IN_COLLISION");
    case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return std::string("START_STATE_VIOLATES_PATH_CONSTRAINTS");
    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
      return std::string("GOAL_IN_COLLISION");
    case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return std::string("GOAL_VIOLATES_PATH_CONSTRAINTS");
    case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
      return std::string("GOAL_CONSTRAINTS_VIOLATED");
    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
      return std::string("INVALID_GROUP_NAME");
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
      return std::string("INVALID_GOAL_CONSTRAINTS");
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
      return std::string("INVALID_ROBOT_STATE");
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
      return std::string("INVALID_LINK_NAME");
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
      return std::string("INVALID_OBJECT_NAME");
    case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
      return std::string("FRAME_TRANSFORM_FAILURE");
    case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
      return std::string("COLLISION_CHECKING_UNAVAILABLE");
    case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
      return std::string("ROBOT_STATE_STALE");
    case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
      return std::string("SENSOR_INFO_STALE");
    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
      return std::string("NO_IK_SOLUTION");
    default:
      return std::string("INVALID MOVEIT CODE");
  }
};

#endif  // MOVEIT_ERROR_CODE_INTERFACE_H