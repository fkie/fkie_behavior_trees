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

#include "Recovery.h"

namespace MoveBaseActionsGroup
{
Recovery::Recovery(const std::string& _name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(_name, config), name("[" + _name + "] ")
{
  ros::NodeHandle private_node("~");

  private_node.param("MoveBaseActions/topic_recovery_action", topic_recovery_action, std::string(""));
  if (topic_recovery_action.empty())
  {
    BT_ROS_ERROR_STREAM(name << "Missing param: [MoveBaseActions/topic_recovery_action]");
    return;
  }

  ac = std::make_unique<ClientRecoveryAction>(topic_recovery_action, true);
  BT_ROS_INFO_STREAM(name << "Waiting for action server [" << topic_recovery_action << "]");
  ac->waitForServer();
  BT_ROS_INFO_STREAM(name << "Action client started");

  init = true;
}

Recovery::~Recovery()
{
  if (ac && ac->isServerConnected())
    ac->cancelAllGoals();
}

BT::NodeStatus Recovery::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  if (!init)
  {
    BT_ROS_ERROR_STREAM(name << "not initialized!");
    return BT::NodeStatus::FAILURE;
  }

  if (!ac->isServerConnected())
  {
    BT_ROS_ERROR("Action server is not connected");
    return BT::NodeStatus::FAILURE;
  }

  std::string strategy;

  auto res = BT::TreeNode::getInput("strategy", strategy);
  if (!res)
  {
    BT_ROS_ERROR_STREAM(name << "Could not load [strategy]: " << res.error());
    return BT::NodeStatus::FAILURE;
  }

  if (strategy.empty())
  {
    BT_ROS_INFO_STREAM(name << "No valid strategy was found");
    return BT::NodeStatus::FAILURE;
  }

  exe_result.store(false);

  BT_ROS_INFO_STREAM(name << "starting recovery: [" << strategy << "]");
  mbf_msgs::RecoveryGoal goal_action;
  goal_action.behavior = strategy;

  if (ac)
  {
    ac->sendGoal(goal_action, boost::bind(&Recovery::doneCallback, this, _1, _2),
                 boost::bind(&Recovery::activeCallback, this), boost::bind(&Recovery::feedbackCallback, this));

    ac->waitForResult();
  }

  BT_ROS_DEBUG_STREAM(name << "Finishing with exe_result: [" << exe_result << "]");
  return exe_result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void Recovery::halt()
{
  exe_result.store(false);
  if (ac)
    ac->cancelAllGoals();
  setStatus(BT::NodeStatus::FAILURE);
}

void Recovery::doneCallback(const actionlib::SimpleClientGoalState& state,
                            const mbf_msgs::RecoveryResultConstPtr& result)
{
  mbf_msgs::RecoveryResult r = *result;
  if (state != actionlib::SimpleClientGoalState::LOST)
  {
    if (r.outcome == mbf_msgs::RecoveryResult::SUCCESS)
    {
      exe_result.store(true);
      BT_ROS_INFO_STREAM(name << "Recovery successfully executed: [" << r.used_plugin << "] message: " << r.message);
    }
    else
    {
      exe_result.store(false);
      BT_ROS_INFO_STREAM(name << "Recovery failed: [" << r.used_plugin << "] message: " << r.message);
    }
  }
  else
  {
    BT_ROS_ERROR_STREAM("Transition to LOST. Check if Action server [" << topic_recovery_action << "] is running.");
  }
}

void Recovery::activeCallback()
{
  // ROS_INFO("Recovery::activeCallback");
}

void Recovery::feedbackCallback()
{
  BT_ROS_DEBUG_STREAM("Not Implemented: Recovery::feedbackCallback");
}
}  // namespace MoveBaseActionsGroup
