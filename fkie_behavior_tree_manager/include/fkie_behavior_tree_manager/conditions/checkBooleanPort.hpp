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

#ifndef CHECK_BOOLEAN_PORT_H
#define CHECK_BOOLEAN_PORT_H

#include <ros/ros.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

namespace BehaviorTreeNodes
{
BT::NodeStatus CheckBooleanPort(BT::TreeNode& self)
{
  bool port_value = false;
  bool success_value = false;

  auto res = self.getInput("port_value", port_value);
  if (!res)
  {
    BT_ROS_ERROR_STREAM(self.name() << " Error at reading [port_value]: " << res.error());
    return BT::NodeStatus::FAILURE;
  }

  res = self.getInput("success_value", success_value);
  if (!res)
  {
    BT_ROS_ERROR_STREAM(self.name() << " Error at reading [success_value]: " << res.error());
    return BT::NodeStatus::FAILURE;
  }

  if (port_value == success_value)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
};

}  // namespace BehaviorTreeNodes

#endif  // CHECK_BOOLEAN_PORT_H