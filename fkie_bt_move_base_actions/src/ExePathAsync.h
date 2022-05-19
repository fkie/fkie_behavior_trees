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

#ifndef MOVE_BASE_EXE_PATH_ASYNC_H
#define MOVE_BASE_EXE_PATH_ASYNC_H

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>

#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/RecoveryAction.h>

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> ClientExePathAsyncAction;

namespace MoveBaseActionsGroup
{
class ExePathAsync : public BT::CoroActionNode
{
public:
  std::string topic_exe_path;
  ros::Subscriber sub_pose, sub_result;
  std::string topic_pose_reset, frame_id_target_pose;

  bool clear_pose_and_path_after_execution = true;
  bool cancel_goals_on_stop = true;

  bool is_initialized = false;

  ExePathAsync(const std::string& name, const BT::NodeConfiguration& config);
  ~ExePathAsync();

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    static BT::PortsList ports = { BT::BidirectionalPort<Pose2D>("target_pose"),
                                   BT::BidirectionalPort<nav_msgs::Path>("path"),
                                   BT::InputPort<std::string>("controller"),
                                   BT::OutputPort<unsigned int>("nav_error_code") };

    return ports;
  }

  bool sendNewGoal();
  void doneCallback(const actionlib::SimpleClientGoalState& state, const mbf_msgs::ExePathResultConstPtr& result);
  void activeCallback();
  void feedbackCallback(const mbf_msgs::ExePathFeedbackConstPtr& feedback);
  void callbackPose(const geometry_msgs::PoseStamped& msg);
  bool checkValidState(const int outcome);

  void halt() override;

private:
  std::atomic_bool exe_result, navigation_finished, must_restart, tracking_goal;
  std::unique_ptr<ClientExePathAsyncAction> ac;
};

}  // namespace MoveBaseActionsGroup

#endif  // MOVE_BASE_EXE_PATH_ASYNC_H
