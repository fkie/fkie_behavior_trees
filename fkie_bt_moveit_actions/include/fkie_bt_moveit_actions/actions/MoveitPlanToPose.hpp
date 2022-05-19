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

#ifndef PLAN_TO_POSE_H
#define PLAN_TO_POSE_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/ros.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <eigen_conversions/eigen_msg.h>
#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "../MoveitErrorCodeInterface.hpp"

namespace MoveitActionsGroup
{
// MoveIt ROS Melodic API change: "Replaced Eigen::Affine3d with Eigen::Isometry3d, which is computationally more
// efficient."
#if ROS_VERSION_MINIMUM(1, 14, 0)
typedef Eigen::Isometry3d EigenTransformType;
#else
typedef Eigen::Affine3d EigenTransformType;
#endif

class MoveitPlanToPose : public BT::SyncActionNode
{
public:
  std::string planning_group;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
  double radius, x_init, y_init, z_init;
  double planning_time;
  bool allow_replanning = true;
  double eef_step = 0.0;
  double jump_threshold = 0.0;
  double position_tolerance = 0.0;
  double orientation_tolerance = 0.0;

  std::string target_frame;
  std::string end_effector_frame;

  ros::Publisher debug_pub;
  ros::Publisher display_publisher;

  std::unique_ptr<tf::TransformListener> listener;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("move_group_plan"),
                            BT::InputPort<geometry_msgs::PoseStamped>("target_pose_3d") };

    return ports;
  }

  MoveitPlanToPose(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
    ros::NodeHandle private_node("~");
    debug_pub = private_node.advertise<geometry_msgs::PoseStamped>("modified_eef_pose", 1);
    display_publisher = private_node.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);
    private_node.param("MoveitActions/planning_group", planning_group, std::string("arm"));
    private_node.param("MoveitActions/planning_time", planning_time, 10.0);
    private_node.param("MoveitActions/allow_replanning", allow_replanning, true);
    private_node.param("MoveitActions/eef_step", eef_step, 0.01);
    private_node.param("MoveitActions/jump_threshold", jump_threshold, 0.0);
    private_node.param("MoveitActions/position_tolerance", position_tolerance, 0.4);
    private_node.param("MoveitActions/orientation_tolerance", orientation_tolerance, 0.3);

    private_node.param("MoveitActions/target_frame", target_frame, std::string(""));
    private_node.param("MoveitActions/end_effector_frame", end_effector_frame, std::string(""));

    BT_ROS_INFO_STREAM("ROSParams of MoveitPlanToPose");

    BT_ROS_INFO_STREAM("MoveitActions/planning_group [" << planning_group << "]");
    BT_ROS_INFO_STREAM("MoveitActions/planning_time [" << planning_time << "]");
    BT_ROS_INFO_STREAM("MoveitActions/allow_replanning [" << allow_replanning << "]");
    BT_ROS_INFO_STREAM("MoveitActions/eef_step [" << eef_step << "]");
    BT_ROS_INFO_STREAM("MoveitActions/jump_threshold [" << jump_threshold << "]");
    BT_ROS_INFO_STREAM("MoveitActions/position_tolerance [" << position_tolerance << "]");
    BT_ROS_INFO_STREAM("MoveitActions/orientation_tolerance [" << orientation_tolerance << "]");
    listener = std::make_unique<tf::TransformListener>();

    if (planning_group.empty())
    {
      BT_ROS_ERROR("Missing param [MoveitActions/planning_group]");
      return;
    }

    // setup using just the name of the planning group you would like to control and plan for.
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(planning_group);
    visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("base_link");
    visual_tools->loadRobotStatePub("/display_robot_state");

    BT_ROS_INFO_STREAM("Available: target poses:");
    for (std::string name : move_group->getNamedTargets())
    {
      BT_ROS_INFO_STREAM(" [" << name << "]");
    }
  }

  ~MoveitPlanToPose()
  {
  }

  BT::NodeStatus tick()
  {
    setStatus(BT::NodeStatus::RUNNING);

    geometry_msgs::PoseStamped target_pose_3d;
    auto res = BT::TreeNode::getInput("target_pose_3d", target_pose_3d);
    if (!res)
    {
      BT_ROS_ERROR_STREAM("Could not load [target_pose_3d]: " << res.error());
      // BT::TreeNode::setOutput("target_pose_3d", target_pose_3d);
      return BT::NodeStatus::FAILURE;
    }

    BT_ROS_INFO_STREAM("Starting arm planning");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    move_group->setPlanningTime(planning_time);
    move_group->allowReplanning(allow_replanning);
    move_group->setGoalPositionTolerance(position_tolerance);
    move_group->setGoalOrientationTolerance(orientation_tolerance);

    move_group->setStartStateToCurrentState();

    // Transformation between [end_effector_frame] (moveit link) and [target_frame]
    EigenTransformType tf_eef_target_frame;
    EigenTransformType requested_pose;

    tf::poseMsgToEigen(target_pose_3d.pose, requested_pose);

    tf::StampedTransform tf_trafo;
    std::string error_string;
    ros::Time now = ros::Time::now();

    if (listener->waitForTransform(target_frame, end_effector_frame, now, ros::Duration(0.3), ros::Duration(0.01),
                                   &error_string))
    {
      listener->lookupTransform(target_frame, end_effector_frame, now, tf_trafo);
    }
    else
    {
      BT_ROS_WARN_STREAM("tf transform not available: " << error_string << "... using _now_-1 instead");
      listener->lookupTransform(target_frame, end_effector_frame, ros::Time::now() - ros::Duration(1.0), tf_trafo);
    }

    tf::transformTFToEigen(tf_trafo, tf_eef_target_frame);

    // Calculate the target pose for the end effector to ensure that the camera reaches the NBV target pose
    EigenTransformType modified_eef_pose = requested_pose * tf_eef_target_frame;
    geometry_msgs::PoseStamped new_eef_pose;
    new_eef_pose.header.frame_id = target_pose_3d.header.frame_id;
    new_eef_pose.header.stamp = ros::Time::now();
    tf::poseEigenToMsg(modified_eef_pose, new_eef_pose.pose);
    debug_pub.publish(new_eef_pose);

    // Set the joint value target for the new end-effector pose
    bool ik_result = move_group->setApproximateJointValueTarget(new_eef_pose, end_effector_frame);
    if (!ik_result)
    {
      BT_ROS_ERROR("IK failed");
      return BT::NodeStatus::FAILURE;
    }
    moveit::core::MoveItErrorCode success = true;
    BT_ROS_DEBUG_STREAM("Executing moveit to go to goal");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    BT_ROS_INFO_STREAM("Plan: " << getMoveitErrorCodeDescription(move_group->plan(plan)));
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = plan.start_state_;
    display_trajectory.trajectory.push_back(plan.trajectory_);
    display_publisher.publish(display_trajectory);

    BT_ROS_INFO_STREAM("Motion execution result: " << getMoveitErrorCodeDescription(success));

    if (!success)
    {
      BT_ROS_INFO_STREAM("Error move_group->execute(plan)");
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      BT_ROS_INFO_STREAM("Publishing plan to blackboard");
      BT::TreeNode::setOutput("move_group_plan", plan);
      ros::Duration(1.0).sleep();
      return BT::NodeStatus::SUCCESS;
    }
  }
};
}  // namespace MoveitActionsGroup
#endif  // PLAN_TO_POSE_H
