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

#ifndef MOVE_TO_POSE_H
#define MOVE_TO_POSE_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/ros.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <fkie_behavior_tree_manager/utils.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

class MoveitMoveToPose : public BT::SyncActionNode
{
private:
  tf2_ros::Buffer tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  double radius, x_init, y_init, z_init;
  ros::Publisher debug_pub;
  ros::Publisher display_publisher;

  std::unique_ptr<tf::TransformListener> listener;

protected:
  // ROS params and corresponding default values
  std::string planning_group = "arm";
  bool execute_trajectory = false;
  double planning_time = 10.0;
  bool allow_replanning = true;
  double position_tolerance = 0.4;
  double orientation_tolerance = 0.3;
  std::string robot_frame;
  std::string target_frame;
  std::string end_effector_frame;

public:
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<geometry_msgs::PoseStamped>("target_pose_3d") };

    return ports;
  }

  MoveitMoveToPose(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
    ros::NodeHandle private_node("~");

    debug_pub = private_node.advertise<geometry_msgs::PoseStamped>("modified_eef_pose", 1);
    display_publisher = private_node.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1, true);

    readRosParameters(name);

    // Initialise the tf buffer to the tf listener
    listener = std::make_unique<tf::TransformListener>();
    tf2_listener = std::make_shared<tf2_ros::TransformListener>(tfBuffer, true);

    if (planning_group.empty())
    {
      BT_ROS_ERROR("Missing param [MoveitActions/planning_group]");
      return;
    }

    // setup using just the name of the planning group you would like to control and plan for.
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(planning_group);

    BT_ROS_INFO_STREAM("Available: target poses:");
    for (std::string name : move_group->getNamedTargets())
    {
      BT_ROS_INFO_STREAM(" [" << name << "]");
    }
  }

  ~MoveitMoveToPose()
  {
  }

  /**
   * @brief Read ROS parameters, considering overwrite parameters if given
   *
   * @param name: BT node name given in the XML file
   */
  void readRosParameters(const std::string& name)
  {
    BT_ROS_INFO_STREAM("[" << name << "]: ROS parameters ----------------------------");

    getROSParameter("MoveitActions/planning_group", planning_group);
    getROSParameter("MoveitActions/execute_trajectory", execute_trajectory);
    getROSParameter("MoveitActions/planning_time", planning_time);
    getROSParameter("MoveitActions/allow_replanning", allow_replanning);
    getROSParameter("MoveitActions/position_tolerance", position_tolerance);
    getROSParameter("MoveitActions/orientation_tolerance", orientation_tolerance);
    getROSParameter("MoveitActions/robot_frame", robot_frame);
    getROSParameter("MoveitActions/target_frame", target_frame);
    getROSParameter("MoveitActions/end_effector_frame", end_effector_frame);

    // check if param must be overwritten
    std::string planning_group_overwrite;
    getROSParameter(name + "/planning_group", planning_group_overwrite);
    if (!planning_group_overwrite.empty())
    {
      planning_group = planning_group_overwrite;
      BT_ROS_INFO_STREAM("Overwritting [planning_group] to: " << planning_group_overwrite);
    }

    std::string target_frame_overwrite;
    getROSParameter(name + "/target_frame", target_frame_overwrite);
    if (!target_frame_overwrite.empty())
    {
      target_frame = target_frame_overwrite;
      BT_ROS_INFO_STREAM("Overwritting [target_frame] to: " << target_frame_overwrite);
    }

    std::string end_effector_frame_overwrite;
    getROSParameter(name + "/end_effector_frame", end_effector_frame_overwrite);
    if (!end_effector_frame_overwrite.empty())
    {
      end_effector_frame = end_effector_frame_overwrite;
      BT_ROS_INFO_STREAM("Overwritting [end_effector_frame] to: " << end_effector_frame_overwrite);
    }

    BT_ROS_INFO_STREAM("----------------------------");
  }

  BT::NodeStatus tick()
  {
    setStatus(BT::NodeStatus::RUNNING);

    geometry_msgs::PoseStamped target_pose_3d;
    auto res = BT::TreeNode::getInput("target_pose_3d", target_pose_3d);
    if (!res)
    {
      BT_ROS_ERROR_STREAM("[MoveitMoveToPose] Could not load [target_pose_3d]: " << res.error());
      // BT::TreeNode::setOutput("target_pose_3d", target_pose_3d);
      return BT::NodeStatus::FAILURE;
    }

    BT_ROS_INFO_STREAM("Starting arm motion");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Transform the goals into planning frame
    geometry_msgs::PoseStamped temp_target_pose_3d;
    temp_target_pose_3d = target_pose_3d;

    if (temp_target_pose_3d.header.frame_id.compare(robot_frame) != 0)
    {
      // BT_ROS_INFO_STREAM("[MoveitMoveToPose] poses frame id is:" << temp_target_pose_3d.header.frame_id);
      // BT_ROS_INFO_STREAM("[MoveitMoveToPose] Planning frame id is:" << robot_frame);
      BT_ROS_INFO_STREAM("[MoveitMoveToPose] Transforming pose from current frame:"
                         << temp_target_pose_3d.header.frame_id << " to planning frame:" << robot_frame);
      target_pose_3d = transformPose(temp_target_pose_3d, robot_frame, temp_target_pose_3d.header.frame_id);
      BT_ROS_INFO_STREAM("[MoveitMoveToPose] Transformed pose for execution: ("
                         << target_pose_3d.pose.position.x << "," << target_pose_3d.pose.position.y << ","
                         << target_pose_3d.pose.position.z << ") with frame_id:" << target_pose_3d.header.frame_id);
    }
    else
    {
      BT_ROS_INFO_STREAM("[MoveitMoveToPose] Received pose for execution in planning frame: ("
                         << target_pose_3d.pose.position.x << "," << target_pose_3d.pose.position.y << ","
                         << target_pose_3d.pose.position.z << ") with frame_id:" << target_pose_3d.header.frame_id);
    }

    move_group->setPlanningTime(planning_time);
    move_group->allowReplanning(allow_replanning);
    move_group->setGoalPositionTolerance(position_tolerance);
    move_group->setGoalOrientationTolerance(orientation_tolerance);
    move_group->setStartStateToCurrentState();

    // Transformation between [end_effector_frame] (moveit link) and [target_frame]
    // Based on Matthias's code in
    // https://gitlab.fkie.fraunhofer.de/cms-npc/simple_pick_and_place/-/blob/eb733024bc10a80926733aa9162abe1d6ad0c721/simple_pick_place_fkie/src/grasp_at_point_execution.cpp
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
    success = move_group->execute(plan);
    BT_ROS_INFO_STREAM("Motion execution result: " << getMoveitErrorCodeDescription(success));

    if (!success)
    {
      BT_ROS_INFO_STREAM("Error move_group->execute(plan)");
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      ros::Duration(1.0).sleep();
      return BT::NodeStatus::SUCCESS;
    }
  }

  /**
   * @brief Transform a posestamped input pose to some frame
   *
   * @param input_pose: Pose in some input frame
   * @param to_frame: Pose in some output frame
   * @param from_frame: Input frame
   * @return geometry_msgs::PoseStamped Transformed: Transformed pose
   */
  geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped input_pose, std::string to_frame,
                                           std::string from_frame)
  {
    geometry_msgs::PoseStamped output_pose;
    geometry_msgs::TransformStamped trans;
    try
    {
      trans = tfBuffer.lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(input_pose, output_pose, trans);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }

    return output_pose;
  }
};
}  // namespace MoveitActionsGroup
#endif  // MOVE_TO_POSE_H
