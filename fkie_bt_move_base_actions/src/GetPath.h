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

#ifndef MOVE_BASE_GET_PATH_H
#define MOVE_BASE_GET_PATH_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/RecoveryAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

typedef actionlib::SimpleActionClient<mbf_msgs::GetPathAction> ClientGetPathAction;
typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> ClientExePathAction;

namespace MoveBaseActionsGroup
{
class GetPath : public BT::SyncActionNode
{
public:
  bool is_initialized = false;
  std::string topic_get_path, frame_id_get_path;
  std::string topic_exe_path;
  std::string frame_id_target_pose;

  tf::TransformListener mytf;

  GetPath(const std::string& name, const BT::NodeConfiguration& config);
  ~GetPath();
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<Pose2D>("target_pose"), BT::InputPort<std::string>("planner"),
                            BT::OutputPort<nav_msgs::Path>("path"), BT::OutputPort<unsigned int>("nav_error_code") };
    return ports;
  }

  void doneCallback(const actionlib::SimpleClientGoalState& state, const mbf_msgs::GetPathResultConstPtr& result);
  void activeCallback();
  void feedbackCallback(const mbf_msgs::GetPathFeedbackConstPtr& feedback);
  bool getTfTransform(const std::string frame_to, const std::string frame_from, tf::StampedTransform& transform);

private:
  std::atomic_bool exe_result;
  std::unique_ptr<ClientGetPathAction> ac;
  std::unique_ptr<ClientExePathAction> ac_exec;
};
}  // namespace MoveBaseActionsGroup
#endif  // MOVE_BASE_GET_PATH_H
