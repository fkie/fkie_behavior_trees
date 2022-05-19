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

#ifndef MOVE_BASE_RECOVERY_H
#define MOVE_BASE_RECOVERY_H

#include <ros/ros.h>

#include <nav_msgs/Path.h>

#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/RecoveryAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/action_server.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

typedef actionlib::SimpleActionClient<mbf_msgs::RecoveryAction> ClientRecoveryAction;

namespace MoveBaseActionsGroup
{
class Recovery : public BT::ActionNodeBase
{
public:
  std::string name;
  std::string topic_recovery_action;
  bool init = false;

  Recovery(const std::string& name, const BT::NodeConfiguration& config);
  ~Recovery();

  static BT::PortsList providedPorts()
  {
    static BT::PortsList ports = { BT::InputPort<std::string>("strategy") };
    return ports;
  }

  BT::NodeStatus tick() override;
  void halt() override;

  void doneCallback(const actionlib::SimpleClientGoalState& state, const mbf_msgs::RecoveryResultConstPtr& result);
  void activeCallback();
  void feedbackCallback();
  bool checkValidState(const int outcome);

private:
  std::atomic_bool exe_result;
  std::unique_ptr<ClientRecoveryAction> ac;
};

}  // namespace MoveBaseActionsGroup

#endif  // MOVE_BASE_RECOVERY_H
