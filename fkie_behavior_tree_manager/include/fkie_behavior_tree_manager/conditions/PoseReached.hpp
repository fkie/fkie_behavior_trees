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

#ifndef POSE_REACHED_NODE_H
#define POSE_REACHED_NODE_H

#include <chrono>
#include <fkie_behavior_tree_manager/logging.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace BehaviorTreeNodes
{
class PoseReached : public BT::ConditionNode
{
public:
  std::string name;
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_global_robot;
  ros::Subscriber sub_pose;
  std::string frame_robot, frame_global;

  // command interface
  std::string command_topic, current_command, reset_command;
  double timeout = 0.0;

  Pose2D target_pose;
  double distance;     // Threshold distance
  double angle;        // Threshold angle
  bool wait_for_pose;  // wait for  pose or return FAIL/SUCCESS immediately
  bool valid_init = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::BidirectionalPort<Pose2D>("target_pose"), BT::InputPort<double>("distance"),
                            BT::InputPort<double>("angle"), BT::InputPort<double>("timeout") };
    return ports;
  }

  PoseReached(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");
    ros::NodeHandle public_node;

    std::string parameter_frame_robot = "PoseReached/" + _name + "_frame_robot";
    std::string parameter_frame_global = "PoseReached/" + _name + "_frame_global";
    std::string parameter_wait_for_pose = "PoseReached/" + _name + "_wait_for_pose";

    reset_command = _name + "_reset";

    private_node.param<std::string>(parameter_frame_robot, frame_robot, std::string(""));
    private_node.param<std::string>(parameter_frame_global, frame_global, std::string(""));
    private_node.param<bool>(parameter_wait_for_pose, wait_for_pose, true);

    BT_ROS_DEBUG_STREAM(name << parameter_wait_for_pose << ": " << wait_for_pose);

    private_node.param<std::string>("PoseReached/command_topic", command_topic, std::string("bt_cmd"));

    if (frame_robot.empty() || frame_global.empty())
    {
      BT_ROS_ERROR_STREAM(name << " [frame_robot] or [frame_global] is empty.");
      return;
    }

    sub_pose = public_node.subscribe(command_topic, 1, &PoseReached::callbackCommand, this);

    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    // get parameters
    auto res = BT::TreeNode::getInput("target_pose", target_pose);
    if (!res)
    {
      BT_ROS_WARN_STREAM(name << "Could not load target_pose, resetting variable. Error: " << res.error());
      BT::TreeNode::setOutput("target_pose", Pose2D());  // reset current target_pose
      return BT::NodeStatus::FAILURE;
    }

    BT::TreeNode::getInput("distance", distance);
    BT::TreeNode::getInput("angle", angle);
    BT::TreeNode::getInput("timeout", timeout);

    if (wait_for_pose)
    {
      // waiting until goal reached
      BT_ROS_INFO_STREAM(name << "waiting for reaching the goal, timeout: " << timeout);
      current_command = "";

      ros::Time begin = ros::Time::now();

      while (ros::ok())
      {
        // check if reset
        if (current_command == reset_command)
        {
          BT::TreeNode::setOutput("target_pose", Pose2D());  // reset current target_pose
          current_command = "";
          BT_ROS_WARN_STREAM(name << "skipping goal and resetting");
          return BT::NodeStatus::FAILURE;
        }

        if (checkPoseReached())
          return BT::NodeStatus::SUCCESS;

        // check timeout
        ros::Duration total_time = ros::Time::now() - begin;
        if (timeout > 0.0 && total_time.toSec() > timeout)
        {
          BT_ROS_WARN_STREAM(name << "timeout of [" << timeout
                                  << "] (s) reached, skipping goal. total_time:" << total_time.toSec());
          return BT::NodeStatus::FAILURE;
        }

        ros::Duration(0.2).sleep();
        ros::spinOnce();
      }

      return BT::NodeStatus::FAILURE;
    }
    else
    {
      if (checkPoseReached())
        return BT::NodeStatus::SUCCESS;
      else
        return BT::NodeStatus::FAILURE;
    }
  }

  void callbackCommand(const std_msgs::String& msg)
  {
    current_command = msg.data;

    BT_ROS_DEBUG_STREAM(name << " new command received: " << current_command);
  }

  bool checkPoseReached()
  {
    // get current position using TF frame robot
    while (ros::ok() && (!tf_listener.waitForTransform(frame_global, frame_robot, ros::Time(), ros::Duration(0.5))))
    {
      BT_ROS_INFO_STREAM(name << "Wait for transform between [" << frame_global << "] and [" << frame_robot << "]");
      ros::Duration(0.2).sleep();
    }
    tf_listener.lookupTransform(frame_global, frame_robot, ros::Time(0), tf_global_robot);

    double current_x = tf_global_robot.getOrigin().x();
    double current_y = tf_global_robot.getOrigin().y();

    tf::Quaternion current_q = tf_global_robot.getRotation();
    double current_angle = tf::getYaw(current_q);

    double distance_delta = std::fabs(current_x - target_pose.x) + std::fabs(current_y - target_pose.y);
    double angle_delta = std::fabs(current_angle) - std::fabs(target_pose.theta);

    bool reached = (distance_delta <= distance && angle_delta <= angle);
    if (reached)
    {
      BT::TreeNode::setOutput("target_pose", Pose2D());  // reset current target_pose
    }

    // ROS_INFO_STREAM("PoseReached: current_x " << current_x << " current_y " << current_y << " target_pose.x " <<
    // target_pose.x << " target_pose.y " << target_pose.y); ROS_INFO_STREAM("PoseReached: distance_delta [" <<
    // distance_delta << "] angle_delta [" << angle_delta << "] reached " << reached);

    return reached;
  }
};

}  // namespace BehaviorTreeNodes

#endif  // POSE_REACHED_NODE_H