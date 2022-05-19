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

#ifndef CARTESIAN_PATH_PLANNING_H
#define CARTESIAN_PATH_PLANNING_H

// Standard ROS imports
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/ros.h>

// Custom ROS imports
#include "../MoveitErrorCodeInterface.hpp"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// BT imports
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// C++ imports
#include <string>

namespace MoveitActionsGroup
{
class MoveitCartesianPathPlanning : public BT::SyncActionNode
{
public:
  std::vector<geometry_msgs::PoseStamped> planned_nbv_poses;
  std::string planning_group_nbv;

  bool execute_trajectory;
  bool allow_replanning;
  std::atomic_bool exe_result;

  double planning_time;

  double jump_threshold;
  double eef_step;
  double position_tolerance = 0.0;
  double orientation_tolerance = 0.0;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  tf2_ros::Buffer tf2Buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf2Listener;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<std::vector<geometry_msgs::PoseStamped>>("planned_nbv_poses") };
    return ports;
  }

  MoveitCartesianPathPlanning(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    ros::NodeHandle private_node("~");
    private_node.param("MoveitActions/planning_group_nbv", planning_group_nbv, std::string("camera"));
    private_node.param("MoveitActions/execute_trajectory", execute_trajectory, false);
    private_node.param("MoveitActions/eef_step", eef_step, 0.01);
    private_node.param("MoveitActions/jump_threshold", jump_threshold, 0.0);
    private_node.param("MoveitActions/planning_time", planning_time, 10.0);
    private_node.param("MoveitActions/allow_replanning", allow_replanning, true);
    private_node.param("MoveitActions/position_tolerance", position_tolerance, 0.4);
    private_node.param("MoveitActions/orientation_tolerance", orientation_tolerance, 0.3);

    tf2Listener = std::make_unique<tf2_ros::TransformListener>(tf2Buffer);

    if (planning_group_nbv.empty())
    {
      BT_ROS_ERROR("[MoveitCartesianPathPlanning]: Missing param [MoveitActions/planning_group_nbv]");
      return;
    }

    // setup using just the name of the planning group you would like to control and plan for.
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(planning_group_nbv);
    BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning]: succesfully initialized");
  }

  ~MoveitCartesianPathPlanning()
  {
  }

  BT::NodeStatus tick()
  {
    setStatus(BT::NodeStatus::RUNNING);

    // Check if the input is not valid
    auto input_poses =
        BT::TreeNode::getInput<std::vector<geometry_msgs::PoseStamped>>("planned_nbv_poses", planned_nbv_poses);
    if (!input_poses)
    {
      BT_ROS_ERROR_STREAM(
          "[MoveitCartesianPathPlanning] : Could not load [planned_nbv_poses]: " << input_poses.error());
      BT::TreeNode::setOutput("planned_nbv_poses", planned_nbv_poses);
      return BT::NodeStatus::FAILURE;
    }

    ros::AsyncSpinner spinner(1);
    spinner.start();

    move_group->setPlanningTime(planning_time);
    move_group->allowReplanning(allow_replanning);
    move_group->setGoalPositionTolerance(position_tolerance);
    move_group->setGoalOrientationTolerance(orientation_tolerance);

    move_group->setStartStateToCurrentState();

    // Verify that the planned_nbv_poses frame id matches with the planning frame
    BT_ROS_DEBUG_STREAM("Planning frame: " << move_group->getPlanningFrame());

    // Cartesian path waypoints
    std::vector<geometry_msgs::Pose> waypoints;

    BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Received [" << planned_nbv_poses.size()
                                                                  << "] poses for execution:");

    for (const geometry_msgs::PoseStamped& p : planned_nbv_poses)
    {
      if (p.header.frame_id.compare(move_group->getPlanningFrame()) != 0)
      {
        BT_ROS_WARN("[MoveitCartesianPathPlanning] Mismatched frameids");
        BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] NBV poses frameid is:" << p.header.frame_id);
        BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Planning frameid is:" << move_group->getPlanningFrame());
        BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Transforming pose from current frame:"
                           << p.header.frame_id << " to planning frame:" << move_group->getPlanningFrame());
        geometry_msgs::PoseStamped new_pose = transformPose(p, move_group->getPlanningFrame(), p.header.frame_id);
        waypoints.push_back(new_pose.pose);
        BT_ROS_INFO_STREAM("Transformed pose: (" << new_pose.pose.position.x << ", " << new_pose.pose.position.y << ", "
                                                 << new_pose.pose.position.z
                                                 << ") with frame_id:" << new_pose.header.frame_id);
      }
      else
      {
        waypoints.push_back(p.pose);
        BT_ROS_INFO_STREAM("     (" << p.pose.position.x << ", " << p.pose.position.y << ", " << p.pose.position.z
                                    << ") - frame_id: [" << p.header.frame_id << "]");
      }
    }

    BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Obtained " << waypoints.size()
                                                                 << " 3D poses for the arm path planning");

    moveit_msgs::RobotTrajectory trajectory;
    bool avoid_collisions = false;
    double fraction =
        move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);
    BT_ROS_DEBUG_STREAM("Fraction from computeCartesianPath:" << fraction);

    if (fraction == -1)
    {
      BT_ROS_ERROR_STREAM("[MoveitCartesianPathPlanning] cartesian path could not be generated.");
      return BT::NodeStatus::FAILURE;
    }

    // Finally plan and execute the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    moveit::core::MoveItErrorCode success = move_group->execute(plan);

    if (!success)
    {
      BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Error when executing plan {move_group->execute(plan)}");
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      BT_ROS_INFO_STREAM("[MoveitCartesianPathPlanning] Plan succesfully executed {move_group->execute(plan)}");
      ros::Duration(1.0).sleep();
      return BT::NodeStatus::SUCCESS;
    }
  }

  geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped input_pose, std::string to_frame,
                                           std::string from_frame)
  {
    geometry_msgs::PoseStamped output_pose;
    geometry_msgs::TransformStamped transformed;
    try
    {
      transformed = tf2Buffer.lookupTransform(to_frame, from_frame, ros::Time(0), ros::Duration(1.0));
      tf2::doTransform(input_pose, output_pose, transformed);
    }
    catch (tf2::TransformException& ex)
    {
      BT_ROS_WARN_STREAM("transformPose error:" << ex.what());
    }
    return output_pose;
  }
};
}  // namespace MoveitActionsGroup
#endif  // CARTESIAN_PATH_PLANNING_H
