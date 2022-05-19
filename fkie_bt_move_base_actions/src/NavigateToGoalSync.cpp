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

#include "NavigateToGoalSync.h"

namespace MoveBaseActionsGroup
{
NavigateToGoalSync::NavigateToGoalSync(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
{
  ros::NodeHandle public_node("");
  ros::NodeHandle private_node("~");

  private_node.param("MoveBaseActions/topic_notify_result", topic_notify_result, std::string("navigation_result"));
  private_node.param("MoveBaseActions/topic_goal_pose_out", topic_goal_pose_out, std::string("goal_pose"));

  if (topic_notify_result.empty())
  {
    BT_ROS_ERROR("Missing param [MoveBaseActions/topic_notify_result]");
    return;
  }

  if (topic_goal_pose_out.empty())
  {
    BT_ROS_ERROR_STREAM("Missing param [MoveBaseActions/topic_goal_pos_out]");
    return;
  }

  pub_pose = public_node.advertise<geometry_msgs::PoseStamped>(topic_goal_pose_out, 10, false);
  sub_notify = public_node.subscribe(topic_notify_result, 1, &NavigateToGoalSync::callbackNavigationResult, this);

  is_initialized = true;
  BT_ROS_INFO_STREAM("Successfully initialized");
}

NavigateToGoalSync::~NavigateToGoalSync()
{
}

void NavigateToGoalSync::halt()
{
  new_message_received = true;
  // force failure
  last_msg.nav_error_code = -1;
  // TODO: Shall we cancel all goals for global and local planner?
}

BT::NodeStatus NavigateToGoalSync::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!is_initialized)
  {
    BT_ROS_ERROR("NavigateToGoalSync is not initialized");
    return BT::NodeStatus::FAILURE;
  }

  BT_ROS_DEBUG_STREAM("NavigateToGoalSync::tick()");

  // send pose to navigation tree --------------------------------------------------------------------------------------
  // get target pose
  Pose2D target_pose;
  auto res = BT::TreeNode::getInput("target_pose", target_pose);
  if (!res)
  {
    BT_ROS_ERROR_STREAM(" Could not load target_pose, resetting variable. Error: " << res.error());
    BT::TreeNode::setOutput("target_pose", Pose2D());  // reset current target_pose
    return BT::NodeStatus::FAILURE;
  }

  if (std::isnan(target_pose.x) || std::isnan(target_pose.y) || std::isnan(target_pose.theta))
  {
    BT_ROS_ERROR_STREAM("Invalid pose: " << target_pose.toString());
    return BT::NodeStatus::FAILURE;
  }

  if (target_pose.frame_id.empty())
  {
    BT_ROS_ERROR_STREAM("Empty frame id: " << target_pose.toString());
    return BT::NodeStatus::FAILURE;
  }

  // publish target pose
  geometry_msgs::PoseStamped pose;
  pose = target_pose.toPoseStamped();
  pub_pose.publish(pose);

  new_message_received = false;

  // wait for feedback --------------------------------------------------------------------------------------
  while (ros::ok() && !new_message_received)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  new_message_received = false;

  BT_ROS_DEBUG_STREAM("NavigateToGoalSync: Navigation finished");

  BT::TreeNode::setOutput("nav_error_code", (unsigned int)last_msg.nav_error_code);
  BT::TreeNode::setOutput("navigation_result", last_msg);
  BT::TreeNode::setOutput("target_pose", Pose2D());  // reset current target_pose

  if (last_msg.nav_error_code == 0)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}

void NavigateToGoalSync::callbackNavigationResult(const fkie_bt_move_base_actions::NavigationResult& msg)
{
  new_message_received = true;
  last_msg = msg;

  BT_ROS_DEBUG_STREAM("New callback navigation result: (" << msg.nav_error_code << ") - " << msg.origin << ": "
                                                          << msg.description);
}

}  // namespace MoveBaseActionsGroup
