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

#ifndef EXECUTE_WAYPOINTS_H
#define EXECUTE_WAYPOINTS_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/ros.h>

#include "../MoveitErrorCodeInterface.hpp"

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>

namespace MoveitActionsGroup
{
class MoveitExecuteWaypoints : public BT::SyncActionNode
{
public:
  std::string moveit_planning_group;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
  double radius, x_init, y_init, z_init;
  bool execute_trajectory = false;

  geometry_msgs::PoseStamped next_cell;
  std::atomic_bool exe_result;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {};
    return ports;
  }

  MoveitExecuteWaypoints(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    ros::NodeHandle private_node("~");
    private_node.param("MoveitActions/planning_group", moveit_planning_group, std::string("arm"));
    private_node.param("MoveitActions/execute_trajectory", execute_trajectory, false);

    private_node.param("MoveitActions/ExecuteWaypoints/radius", radius, 0.0);
    private_node.param("MoveitActions/ExecuteWaypoints/x_init", x_init, 0.0);
    private_node.param("MoveitActions/ExecuteWaypoints/y_init", y_init, 0.0);
    private_node.param("MoveitActions/ExecuteWaypoints/z_init", z_init, 0.0);

    if (moveit_planning_group.empty())
    {
      BT_ROS_ERROR("MoveitExecuteWaypoints: Missing param [MoveitActions/planning_group]");
      return;
    }

    // setup using just the name of the planning group you would like to control and plan for.
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(moveit_planning_group);
    visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("base_link");

    BT_ROS_INFO_STREAM("succesfully initialized");
  }

  ~MoveitExecuteWaypoints()
  {
  }

  BT::NodeStatus tick()
  {
    setStatus(BT::NodeStatus::RUNNING);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    BT_ROS_INFO_STREAM("Starting arm motion to fixed poses with radius: " << radius);

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(moveit_planning_group);

    // ROS_INFO_STREAM("Reference frame: " << move_group->getPlanningFrame());
    // ROS_INFO_STREAM("End effector link: " << move_group->getEndEffectorLink());

    std::vector<Pose2D> relative_poses;
    relative_poses.push_back(Pose2D(radius, radius));
    relative_poses.push_back(Pose2D(radius, -radius));
    relative_poses.push_back(Pose2D(-radius, -radius));
    relative_poses.push_back(Pose2D(-radius, radius));

    // Cartesian Paths
    geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints;

    for (Pose2D p : relative_poses)
    {
      geometry_msgs::Pose target = current_pose;
      target.position.x = x_init + p.x;
      target.position.y = y_init + p.y;
      target.position.z = z_init;
      waypoints.push_back(target);
    }

    BT_ROS_DEBUG_STREAM("Computing cartesian path for [" << waypoints.size() << "] poses");

    waypoints.push_back(waypoints[0]);  // close loop

    move_group->setMaxVelocityScalingFactor(0.1);
    move_group->setPlanningTime(10);
    move_group->allowReplanning(true);

    moveit_msgs::RobotTrajectory trajectory;
    move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    BT_ROS_DEBUG_STREAM("Cartesian path computed succesfully ");

    // The trajectory needs to be modified so it will include velocities
    robot_trajectory::RobotTrajectory rt(move_group->getCurrentState()->getRobotModel(), moveit_planning_group);

    // get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

    // create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iterative_ptp;

    BT_ROS_DEBUG_STREAM("Computing time stamps for trajectory");

    // compute computeTimeStamps
    iterative_ptp.computeTimeStamps(rt);

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);

    // Finally plan and execute the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    moveit::core::MoveItErrorCode success = true;

    if (execute_trajectory)
    {
      BT_ROS_DEBUG_STREAM("Executing trajectory");
      success = move_group->execute(plan);
      BT_ROS_INFO_STREAM("Motion execution result: " << getMoveitErrorCodeDescription(success));
    }
    else
    {
      visual_tools->deleteAllMarkers();
      visual_tools->setManualSceneUpdating(true);
      visual_tools->publishTrajectoryLine(plan.trajectory_, joint_model_group);
      visual_tools->triggerPlanningSceneUpdate();

      ros::Duration(5.0).sleep();
    }

    if (!success)
    {
      BT_ROS_INFO_STREAM("Error when executing plan {move_group->execute(plan)}");
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      BT_ROS_INFO_STREAM("Plan succesfully executed {move_group->execute(plan)}");
      ros::Duration(1.0).sleep();
      return BT::NodeStatus::SUCCESS;
    }
  }
};
}  // namespace MoveitActionsGroup
#endif  // EXECUTE_WAYPOINTS_H