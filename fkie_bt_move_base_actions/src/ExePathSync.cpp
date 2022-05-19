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

#include "ExePathSync.h"

namespace MoveBaseActionsGroup
{
ExePathSync::ExePathSync(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
{
  // Get ros parameters
  ros::NodeHandle private_node("~");
  ros::NodeHandle public_node;
  private_node.param("MoveBaseActions/topic_exe_path", topic_exe_path, std::string("exe_path"));

  // Global Parameters for Move Base Actions
  private_node.param<std::string>("MoveBaseActions/topic_pose_reset", topic_pose_reset, std::string(""));
  private_node.param<std::string>("MoveBaseActions/frame_id_target_pose", frame_id_target_pose, std::string(""));
  private_node.param<bool>("MoveBaseActions/wait_for_reach_goal", wait_for_reach_goal, true);
  private_node.param<bool>("MoveBaseActions/clear_pose_and_path_after_execution", clear_pose_and_path_after_execution,
                           true);
  private_node.param<bool>("MoveBaseActions/publish_confirmation", publish_confirmation, false);
  private_node.param<std::string>("MoveBaseActions/topic_confirmation", topic_confirmation, std::string(""));
  private_node.param("MoveBaseActions/topic_exe_path", topic_exe_path, std::string(""));

  // Overwrite name-dependent parameters
  if (private_node.hasParam("MoveBaseActions/" + name + "/clear_pose_and_path_after_execution"))
    private_node.param<bool>("MoveBaseActions/" + name + "/clear_pose_and_path_after_execution",
                             clear_pose_and_path_after_execution, clear_pose_and_path_after_execution);

  if (topic_exe_path.empty())
  {
    BT_ROS_ERROR("Missing param [MoveBaseActions/topic_exe_path]");
    return;
  }

  if (publish_confirmation && topic_confirmation.empty())
  {
    BT_ROS_ERROR("Missing param [MoveBaseActions/topic_confirmation]");
    return;
  }

  bool spin_thread = false;
  ac = std::make_unique<ClientExePathSyncAction>(topic_exe_path, spin_thread);
  BT_ROS_INFO_STREAM("Waiting for action server [" << topic_exe_path << "]");

  while (ros::ok() && !ac->waitForServer(ros::Duration(0.1)))
  {
    ros::spinOnce();
  }
  BT_ROS_INFO_STREAM("Action client started");

  if (publish_confirmation)
    pub_bool = public_node.advertise<std_msgs::Bool>(topic_confirmation, 10, false);

  sub_result = public_node.subscribe(topic_exe_path + "/result", 1, &ExePathSync::callbackNavigationResult, this);

  sub_pose = public_node.subscribe(topic_pose_reset, 1, &ExePathSync::callbackPose, this);

  is_initialized = true;
  exploration_finished.store(false);
  must_restart.store(false);
  tracking_goal.store(false);

  BT_ROS_INFO_STREAM("Successfully initialized");
}

ExePathSync::~ExePathSync()
{
  sub_pose.shutdown();

  if (publish_confirmation)
    sub_result.shutdown();
}

bool ExePathSync::sendNewGoal()
{
  nav_msgs::Path path;
  auto res = BT::TreeNode::getInput("path", path);
  if (!res)
  {
    BT_ROS_ERROR_STREAM("Could not load [path]: " << res.error());
    return false;
  }

  std::string controller;
  res = BT::TreeNode::getInput("controller", controller);
  if (!res)
  {
    BT_ROS_ERROR_STREAM("Could not load [controller]: " << res.error());
    return false;
  }

  if (path.poses.size() == 0)
  {
    BT_ROS_ERROR("No valid [path] with size = 0");
    return false;
  }

  if (controller.empty())
  {
    BT_ROS_ERROR("No valid empty [controller]");
    return false;
  }

  if (!ac->isServerConnected())
  {
    BT_ROS_ERROR("Action server is not connected");
    return false;
  }

  BT_ROS_INFO_STREAM("Starting execution of a path with [" << path.poses.size() << "] poses");

  mbf_msgs::ExePathGoal goal_action;
  goal_action.path = path;
  goal_action.controller = controller;

  ac->sendGoal(goal_action, boost::bind(&ExePathSync::doneCallback, this, _1, _2),
               boost::bind(&ExePathSync::activeCallback, this), boost::bind(&ExePathSync::feedbackCallback, this, _1));

  tracking_goal.store(true);
  exploration_finished.store(false);

  if (wait_for_reach_goal)
  {
    BT_ROS_DEBUG_STREAM("Waiting for reaching the goal");
    while (ros::ok() && !exploration_finished)
    {
      ros::spinOnce();
      ros::Duration(0.2).sleep();
    }
  }
  else
  {
    exe_result.store(true);
  }

  // ac->waitForResult(); // wait for setting the goal
  tracking_goal.store(false);
  return true;
}

BT::NodeStatus ExePathSync::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  // Restart exploration when new goal received
  if (must_restart)
  {
    must_restart.store(false);
    return BT::NodeStatus::SUCCESS;
  }

  if (!sendNewGoal())
    return BT::NodeStatus::FAILURE;

  return exe_result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void ExePathSync::halt()
{
  BT_ROS_INFO_STREAM("ExePathSync (halt): Stopping local navigation");

  if (ac->isServerConnected())
  {
    ac->cancelAllGoals();
  }

  exploration_finished.store(false);
  must_restart.store(false);
  tracking_goal.store(false);

  setStatus(BT::NodeStatus::FAILURE);
}

void ExePathSync::doneCallback(const actionlib::SimpleClientGoalState& state,
                               const mbf_msgs::ExePathResultConstPtr& result)
{
  mbf_msgs::ExePathResult r = *result;
  BT_ROS_DEBUG_STREAM("doneCallback: state [" << state.toString() << "] message " << r.message);

  if (state != actionlib::SimpleClientGoalState::LOST)
  {
    bool valid_status = checkValidState(r.outcome);
    exe_result.store(valid_status);

    BT::TreeNode::setOutput("nav_error_code", r.outcome);

    if (valid_status)
    {
      BT_ROS_INFO_STREAM("Path execution finished successfully");

      if (wait_for_reach_goal && clear_pose_and_path_after_execution)
      {
        BT::TreeNode::setOutput("path", nav_msgs::Path());  // reset current path
        BT::TreeNode::setOutput("target_pose", Pose2D());   // reset current target_pose
      }
    }
    exploration_finished.store(true);
  }
  else
  {
    BT_ROS_ERROR_STREAM("Transition to LOST. Check if Action server [" << topic_exe_path << "] is running.");
  }
}

void ExePathSync::activeCallback()
{
}

void ExePathSync::feedbackCallback(const mbf_msgs::ExePathFeedbackConstPtr& feedback)
{
  mbf_msgs::ExePathFeedback f = *feedback;

  bool valid_status = checkValidState(f.outcome);
  exe_result.store(valid_status);

  if (!valid_status)
  {
    BT_ROS_DEBUG_STREAM("Finishing exploration: feedbackCallback " << f.outcome << " message " << f.message);
    exploration_finished.store(true);
  }
}

void ExePathSync::callbackPose(const geometry_msgs::PoseStamped& msg)
{
  BT_ROS_INFO("[pose_reset] received, stopping execution and resetting [target_pose]!");

  // cancel all goals if frame_id is empty
  if (msg.header.frame_id.empty())
  {
    ac->cancelAllGoals();
  }

  if (tracking_goal)
    ac->stopTrackingGoal();

  BT::TreeNode::setOutput("target_pose", Pose2D());  // reset current target_pose

  exploration_finished.store(true);
  exe_result.store(true);  // return SUCCESS to avoid launching recovery behaviors
}

void ExePathSync::callbackNavigationResult(const mbf_msgs::ExePathActionResult& msg)
{
  bool result = msg.result.outcome == mbf_msgs::ExePathResult::SUCCESS;
  std_msgs::Bool msg_out;
  msg_out.data = result;
  pub_bool.publish(msg_out);

  BT_ROS_DEBUG_STREAM("New callback navigation result, result: (ExePathResult::SUCCESS) " << result);
}

bool ExePathSync::checkValidState(const int outcome)
{
  bool valid_status = false;

  switch (outcome)
  {
    case mbf_msgs::ExePathResult::SUCCESS: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::SUCCESS");
      valid_status = true;  // exploration finished successfully
      break;
    }
    case mbf_msgs::ExePathResult::FAILURE: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::FAILURE");
      break;
    }
    case mbf_msgs::ExePathResult::CANCELED: {
      // ROS_WARN_STREAM("mbf_msgs::ExePathSync::CANCELED");
      // exploration was canceled by user, counts as successfully finished
      // to avoid recovery behaviors
      BT_ROS_DEBUG_STREAM("ExePathSync: Exploration cancelled");
      valid_status = true;
      break;
    }
    case mbf_msgs::ExePathResult::NO_VALID_CMD: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::NO_VALID_CMD");
      break;
    }
    case mbf_msgs::ExePathResult::PAT_EXCEEDED: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::PAT_EXCEEDED");
      break;
    }
    case mbf_msgs::ExePathResult::COLLISION: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::COLLISION");
      break;
    }
    case mbf_msgs::ExePathResult::OSCILLATION: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::OSCILLATION");
      break;
    }
    case mbf_msgs::ExePathResult::ROBOT_STUCK: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::ROBOT_STUCK");
      break;
    }
    case mbf_msgs::ExePathResult::MISSED_GOAL: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::MISSED_GOAL");
      break;
    }
    case mbf_msgs::ExePathResult::MISSED_PATH: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::MISSED_PATH");
      break;
    }
    case mbf_msgs::ExePathResult::BLOCKED_PATH: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::BLOCKED_PATH");
      break;
    }
    case mbf_msgs::ExePathResult::INVALID_PATH: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::INVALID_PATH");
      break;
    }
    case mbf_msgs::ExePathResult::TF_ERROR: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::TF_ERROR");
      break;
    }
    case mbf_msgs::ExePathResult::NOT_INITIALIZED: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::NOT_INITIALIZED");
      break;
    }
    case mbf_msgs::ExePathResult::INVALID_PLUGIN: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::INVALID_PLUGIN");
      break;
    }
    case mbf_msgs::ExePathResult::INTERNAL_ERROR: {
      BT_ROS_DEBUG_STREAM("mbf_msgs::ExePathSync::INTERNAL_ERROR");
      break;
    }
    default: {
      BT_ROS_DEBUG_STREAM("Unknown error [" << outcome << "] at executing path");
      break;
    }
  }

  return valid_status;
}
}  // namespace MoveBaseActionsGroup
