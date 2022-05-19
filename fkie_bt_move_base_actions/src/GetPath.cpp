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

#include "GetPath.h"

namespace MoveBaseActionsGroup
{
GetPath::GetPath(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
{
  ros::NodeHandle private_node("~");

  private_node.param("MoveBaseActions/topic_get_path", topic_get_path, std::string("get_path"));
  private_node.param("MoveBaseActions/topic_exe_path", topic_exe_path, std::string("exe_path"));
  private_node.param<std::string>("MoveBaseActions/frame_id_target_pose", frame_id_target_pose, std::string(""));

  if (topic_get_path.empty())
  {
    BT_ROS_ERROR("Missing param [MoveBaseActions/topic_get_path]");
    return;
  }

  if (topic_exe_path.empty())
  {
    BT_ROS_ERROR("Missing param [MoveBaseActions/topic_exe_path]");
    return;
  }

  ac = std::make_unique<ClientGetPathAction>(topic_get_path, true);
  BT_ROS_INFO_STREAM("Waiting for action server [" << topic_get_path << "]");
  ac->waitForServer();
  BT_ROS_INFO_STREAM("Action client started");

  ac_exec = std::make_unique<ClientExePathAction>(topic_exe_path, true);
  BT_ROS_INFO_STREAM("Waiting for action server [" << topic_exe_path << "]");
  ac_exec->waitForServer();

  is_initialized = true;

  BT_ROS_INFO_STREAM("Successfully initialized");
}

GetPath::~GetPath()
{
}

BT::NodeStatus GetPath::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!is_initialized)
  {
    BT_ROS_ERROR("GetPath is not initialized");
    return BT::NodeStatus::FAILURE;
  }

  if (!ac->isServerConnected())
  {
    BT_ROS_ERROR("Action server is not connected");
    return BT::NodeStatus::FAILURE;
  }

  std::string planner;
  auto res = BT::TreeNode::getInput("planner", planner);
  if (!res || planner.empty())
  {
    BT_ROS_ERROR_STREAM("Could not load [planner]: " << res.error());
    return BT::NodeStatus::FAILURE;
  }

  Pose2D target_pose;
  res = BT::TreeNode::getInput("target_pose", target_pose);
  if (!res)
  {
    BT_ROS_ERROR_STREAM("Could not load [target_pose]: " << res.error());
    return BT::NodeStatus::FAILURE;
  }

  if (target_pose.frame_id.empty())
  {
    BT_ROS_INFO_STREAM("[target_pose] is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (std::isnan(target_pose.theta))
  {
    BT_ROS_DEBUG_STREAM("[target_pose]: Theta is NAN, setting it to 0.0");
    target_pose.theta = 0.0;
  }

  // convert frame if necessary
  if (!frame_id_target_pose.empty() && frame_id_target_pose != target_pose.frame_id)
  {
    BT_ROS_DEBUG_STREAM("Frame transformation required from [" << target_pose.frame_id << "] to ["
                                                               << frame_id_target_pose << "]");
    BT_ROS_DEBUG_STREAM("pose before transformation: " << target_pose.frame_id << " at " << target_pose.toString());

    tf::StampedTransform transform;
    if (!getTfTransform(frame_id_target_pose, target_pose.frame_id, transform))
      return BT::NodeStatus::FAILURE;

    tf::Pose tfspose;
    tf::Quaternion q;

    tfspose.setOrigin(tf::Vector3(target_pose.x, target_pose.y, 0.0));
    q.setRPY(0, 0, target_pose.theta);
    tfspose.setRotation(q);

    tf::Pose tfpose = transform * tfspose;

    target_pose.frame_id = frame_id_target_pose;
    target_pose.x = tfpose.getOrigin().x();
    target_pose.y = tfpose.getOrigin().y();
    target_pose.theta = tf::getYaw(tfpose.getRotation());
    BT_ROS_DEBUG_STREAM("pose after transformation: " << target_pose.frame_id << " at " << target_pose.toString());
  }

  mbf_msgs::GetPathGoal goal_action;
  goal_action.planner = planner;
  goal_action.use_start_pose = false;
  goal_action.target_pose = target_pose.toPoseStamped();

  ac->sendGoal(goal_action, boost::bind(&GetPath::doneCallback, this, _1, _2),
               boost::bind(&GetPath::activeCallback, this), boost::bind(&GetPath::feedbackCallback, this, _1));

  ac->waitForResult();

  // cancel current exploration if any
  if (!exe_result)
    ac_exec->cancelAllGoals();

  return exe_result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void GetPath::doneCallback(const actionlib::SimpleClientGoalState& state, const mbf_msgs::GetPathResultConstPtr& result)
{
  mbf_msgs::GetPathResult r = *result;

  BT_ROS_DEBUG_STREAM("doneCallback: state " << state.toString() << " message " << r.message);

  if (state != actionlib::SimpleClientGoalState::LOST)
  {
    BT::TreeNode::setOutput("nav_error_code", r.outcome);

    if (r.outcome == mbf_msgs::GetPathResult::SUCCESS)
    {
      auto res = BT::TreeNode::setOutput("path", r.path);
      if (!res)
      {
        BT_ROS_ERROR_STREAM("Could not set the variable [path]: " << res.error());
        exe_result.store(false);
      }
      else
      {
        BT_ROS_INFO_STREAM("A path with [" << r.path.poses.size() << "] poses was computed");
        exe_result.store(true);
      }
    }
    else
    {
      exe_result.store(false);
      BT_ROS_ERROR_STREAM("Resetting target_pose");
      BT_ROS_ERROR_STREAM("Finished in state " << state.toString() << " message " << r.message);
    }
  }
  else
  {
    BT_ROS_ERROR_STREAM("Transition to LOST. Check if Action server [" << topic_get_path << "] is running.");
  }
}

void GetPath::activeCallback()
{
  BT_ROS_DEBUG("activeCallback");
}

void GetPath::feedbackCallback(const mbf_msgs::GetPathFeedbackConstPtr& feedback)
{
  // MBF Does not provide feedback for get_path
}

bool GetPath::getTfTransform(const std::string frame_to, const std::string frame_from, tf::StampedTransform& transform)
{
  transform.frame_id_ = "";
  transform.child_frame_id_ = "";

  if (frame_from == frame_to)
  {
    transform.setIdentity();
    transform.frame_id_ = frame_from;
    transform.child_frame_id_ = frame_to;
    return true;
  }

  if (!mytf.waitForTransform(frame_to, frame_from, ros::Time(0), ros::Duration(1.0)))
  {
    BT_ROS_ERROR_STREAM("waitForTransform failed: frame_to: [" << frame_to << "] frame_from: [" << frame_from << "]");
    return false;
  }

  mytf.lookupTransform(frame_to, frame_from, ros::Time(0), transform);

  return true;
}

}  // namespace MoveBaseActionsGroup
