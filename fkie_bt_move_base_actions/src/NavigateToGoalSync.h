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

#ifndef MOVE_BASE_NAVIGATE_TO_GOAL_SYNC_H
#define MOVE_BASE_NAVIGATE_TO_GOAL_SYNC_H

#include <ros/ros.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <fkie_bt_move_base_actions/NavigationResult.h>

namespace MoveBaseActionsGroup
{
/*

Publishes a goal to the bt_navigation, and wait for a [fkie_bt_move_base_actions/NavigationResult] message.
Used to encapsulate the navigation from other applications
It returns [nav_error_code] > 0 if navigation errors occurred

*/
class NavigateToGoalSync : public BT::ActionNodeBase
{
public:
  bool is_initialized = false;
  bool new_message_received = false;
  std::string topic_goal_pose_out;
  std::string topic_notify_result;

  ros::Publisher pub_pose;

  NavigateToGoalSync(const std::string& name, const BT::NodeConfiguration& config);
  ~NavigateToGoalSync();
  BT::NodeStatus tick() override;
  virtual void halt() override;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<Pose2D>("target_pose"), BT::OutputPort<unsigned int>("nav_error_code"),
                            BT::OutputPort<fkie_bt_move_base_actions::NavigationResult>("navigation_result") };
    return ports;
  }

  void callbackNavigationResult(const fkie_bt_move_base_actions::NavigationResult& msg);

private:
  ros::Subscriber sub_notify;
  fkie_bt_move_base_actions::NavigationResult last_msg;
};
}  // namespace MoveBaseActionsGroup
#endif  // MOVE_BASE_NAVIGATE_TO_GOAL_SYNC_H
