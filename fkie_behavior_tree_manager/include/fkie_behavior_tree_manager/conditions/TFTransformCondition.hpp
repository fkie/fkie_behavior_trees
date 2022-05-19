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

#ifndef TF_TRANSFORM_CONDITION_H
#define TF_TRANSFORM_CONDITION_H

#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf2_ros/transform_listener.h>

#include <fkie_behavior_tree_manager/logging.h>

namespace BehaviorTreeNodes
{
/**
 * @brief Check if a valid TF between [from_frame] and [to_frame] is given.
 *          return SUCCESS if the TF is valid, FAILURE if it can not transform.
 * ROS Params:
 *    [name]/from_frame: source TF frame
 *    [name]/to_frame: target TF frame
 *    [name]/timeout:  TF Timeout
 */
class TFTransformCondition : public BT::ConditionNode
{
public:
  std::string name;

  std::string from_frame;
  std::string to_frame;
  double timeout = 0.0;

  tf2_ros::Buffer tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  bool valid_init = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {};
    return ports;
  }

  TFTransformCondition(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;

    tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer);

    private_node.param<double>(_name + "/timeout", timeout, 1.0);
    private_node.param<std::string>(_name + "/from_frame", from_frame, "");
    private_node.param<std::string>(_name + "/to_frame", to_frame, "");

    BT_ROS_INFO_STREAM(name << " from_frame: " << from_frame);
    BT_ROS_INFO_STREAM(name << " to_frame: " << to_frame);
    BT_ROS_INFO_STREAM(name << " timeout: " << timeout);

    if (from_frame.empty())
    {
      BT_ROS_ERROR_STREAM(name << " [from_frame] is empty, check parameter:" << _name + "/from_frame");
      return;
    }

    if (to_frame.empty())
    {
      BT_ROS_ERROR_STREAM(name << " [to_frame] is empty, check parameter:" << _name + "/to_frame");
      return;
    }

    valid_init = true;
    BT_ROS_INFO_STREAM(name << " initialized");
  }

  BT::NodeStatus tick()
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    bool can_transform = false;

    if (tf_buffer.canTransform(to_frame, from_frame, ros::Time(0), ros::Duration(timeout)))
    {
      try
      {
        geometry_msgs::TransformStamped ts;
        ts = tf_buffer.lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(timeout));
        can_transform = true;
      }
      catch (std::exception& e)
      {
      }
    }

    if (!can_transform)
    {
      BT_ROS_ERROR_STREAM(name << " no transform between [" << from_frame << "] and [" << to_frame << "]");
    }

    return can_transform ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace BehaviorTreeNodes

#endif  // TF_TRANSFORM_CONDITION_H
