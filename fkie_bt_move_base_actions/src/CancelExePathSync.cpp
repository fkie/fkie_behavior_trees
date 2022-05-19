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

#include "CancelExePathSync.h"

namespace MoveBaseActionsGroup
{
CancelExePathSync::CancelExePathSync(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
{
  // Get ros parameters
  ros::NodeHandle private_node("~");
  private_node.param("MoveBaseActions/topic_exe_path", topic_exe_path, std::string("exe_path"));
  private_node.param("MoveBaseActions/topic_exe_path", topic_exe_path, std::string(""));

  if (topic_exe_path.empty())
  {
    BT_ROS_ERROR("Missing param [MoveBaseActions/topic_exe_path]");
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

  is_initialized = true;
  BT_ROS_INFO_STREAM("Successfully initialized");
}

CancelExePathSync::~CancelExePathSync()
{
  cancelGoals();
}

BT::NodeStatus CancelExePathSync::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  cancelGoals();

  return BT::NodeStatus::SUCCESS;
}

void CancelExePathSync::halt()
{
  cancelGoals();

  setStatus(BT::NodeStatus::FAILURE);
}

void CancelExePathSync::cancelGoals()
{
  if (ac->isServerConnected())
  {
    ac->cancelAllGoals();
  }

  BT::TreeNode::setOutput("path", nav_msgs::Path());  // reset current path
  BT::TreeNode::setOutput("target_pose", Pose2D());   // reset current target_pose
  BT_ROS_DEBUG_STREAM("CancelExePathSync: all goals were cancelled");
}

void CancelExePathSync::activeCallback()
{
}

void CancelExePathSync::feedbackCallback(const mbf_msgs::ExePathFeedbackConstPtr& feedback)
{
}

}  // namespace MoveBaseActionsGroup
