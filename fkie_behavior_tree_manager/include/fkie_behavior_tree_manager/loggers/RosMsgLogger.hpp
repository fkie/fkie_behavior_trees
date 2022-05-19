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

#ifndef BT_ROS_MSG_LOGGER_H
#define BT_ROS_MSG_LOGGER_H

#include <ros/ros.h>

#include "fkie_behavior_tree_msgs/BTNodeStatus.h"
#include "fkie_behavior_tree_manager/logging.h"
#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <cstring>

namespace BehaviorTreeNodes
{
class RosMsgLogger : public BT::StatusChangeLogger
{
protected:
  ros::Publisher pub_bt_status;
  fkie_behavior_tree_msgs::BTNodeStatus status_msg;

public:
  RosMsgLogger(const BT::Tree& tree, const std::string topic) : BT::StatusChangeLogger(tree.rootNode())
  {
    ros::NodeHandle nh("~");
    pub_bt_status = nh.advertise<fkie_behavior_tree_msgs::BTNodeStatus>(topic, 10, false);
  }

  void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status, BT::NodeStatus status)
  {
    status_msg.header.stamp = ros::Time::now();
    status_msg.node_name = node.name();
    status_msg.node_id = node.registrationName();
    status_msg.node_type = toStr(node.type());
    status_msg.node_status = toStr(status, false);
    pub_bt_status.publish(status_msg);

    BT_ROS_DEBUG_STREAM("New callback: Node [" << node.name() << "]: " << toStr(prev_status, true) << " -> "
                                               << toStr(status, true));
  }

  void flush() override
  {
  }
};

}  // namespace BehaviorTreeNodes

#endif  // BT_ROS_MSG_LOGGER_H
