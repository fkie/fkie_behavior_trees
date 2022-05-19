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

#ifndef CHECK_POSE_REACHED_NODE_H
#define CHECK_POSE_REACHED_NODE_H

#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <tf2_ros/transform_listener.h>

namespace BehaviorTreeNodes
{
class CheckPoseReachedAsync : public BT::CoroActionNode
{
public:
  std::string name;

  tf2_ros::Buffer tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::string frame_robot, frame_global;

  Pose2D target_pose;
  double distance;  // Threshold distance
  double angle;     // Threshold angle
  bool valid_init = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::BidirectionalPort<Pose2D>("target_pose"), BT::InputPort<double>("distance"),
                            BT::InputPort<double>("angle") };
    return ports;
  }

  CheckPoseReachedAsync(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::CoroActionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;

    tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer);

    std::string parameter_frame_robot = "CheckPoseReachedAsync/" + _name + "_frame_robot";
    std::string parameter_frame_global = "CheckPoseReachedAsync/" + _name + "_frame_global";

    private_node.param<std::string>(parameter_frame_robot, frame_robot, std::string(""));
    private_node.param<std::string>(parameter_frame_global, frame_global, std::string(""));

    if (frame_robot.empty() || frame_global.empty())
    {
      BT_ROS_ERROR_STREAM(name << " [frame_robot] or [frame_global] is empty.");
      return;
    }

    valid_init = true;
    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    // get parameters
    BT::TreeNode::getInput("target_pose", target_pose);
    BT::TreeNode::getInput("distance", distance);
    BT::TreeNode::getInput("angle", angle);

    if (distance <= 0)
    {
      // disable distance threshold, always running...
      return BT::NodeStatus::RUNNING;
    }

    if (checkPoseReached())
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    BT_ROS_DEBUG_STREAM(name << "node has been halted!");
    setStatus(BT::NodeStatus::FAILURE);
    CoroActionNode::halt();
  }

  bool checkPoseReached()
  {
    if (tf_buffer.canTransform(frame_global, frame_robot, ros::Time(0), ros::Duration(2.0)))
    {
      try
      {
        geometry_msgs::TransformStamped ts;
        ts = tf_buffer.lookupTransform(frame_global, frame_robot, ros::Time(0), ros::Duration(2.0));

        double current_x = ts.transform.translation.x;
        double current_y = ts.transform.translation.y;
        double euclidean_distance =
            std::sqrt(std::pow(current_x - target_pose.x, 2) + std::pow(current_y - target_pose.y, 2));

        double current_angle = tf2::getYaw(ts.transform.rotation);

        // compute the absolute distance between two angles

        // option 1:
        double angle_distance =
            M_PI - std::fabs(std::fmod(std::fabs(current_angle - target_pose.theta), 2 * M_PI) - M_PI);

        // option 2:
        // double angle_distance = std::fabs(std::atan2(std::sin(current_angle - target_pose.theta),
        // std::cos(current_angle - target_pose.theta)));

        BT_ROS_DEBUG_STREAM(name << "distance: " << euclidean_distance << " (" << distance
                                 << "), angle: " << angle_distance << " (" << angle << ")");

        return (euclidean_distance <= distance && angle_distance <= angle);
      }
      catch (std::exception& e)
      {
        BT_ROS_ERROR_STREAM(name << "can't transform between [" << frame_global << "] and [" << frame_robot
                                 << "]: " << e.what());
        return false;
      }
    }
    else
    {
      BT_ROS_ERROR_STREAM(name << "can't transform between [" << frame_global << "] and [" << frame_robot << "]");
      return false;
    }
  }
};

}  // namespace BehaviorTreeNodes

#endif  // CHECK_POSE_REACHED_NODE_H