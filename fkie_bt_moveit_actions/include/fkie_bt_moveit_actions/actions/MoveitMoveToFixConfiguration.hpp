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

#ifndef MOVE_TO_NAMED_GOAL_H
#define MOVE_TO_NAMED_GOAL_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/ros.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <fkie_behavior_tree_manager/utils.hpp>

#include "../MoveitErrorCodeInterface.hpp"

namespace MoveitActionsGroup
{
class MoveitMoveToFixConfiguration : public BT::ActionNodeBase
{
protected:
  // ROS params and corresponding default values
  std::string planning_group = "arm";
  bool execute_trajectory = false;
  double planning_time = 10.0;
  bool allow_replanning = true;
  double position_tolerance = 0.4;
  double orientation_tolerance = 0.3;
  std::string target_frame;
  std::string end_effector_frame;

private:
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
  double radius, x_init, y_init, z_init;

public:
  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::InputPort<std::string>("goal_name") };
    return ports;
  }

  MoveitMoveToFixConfiguration(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
  {
    readRosParameters(name);

    if (planning_group.empty())
    {
      BT_ROS_ERROR("Missing param [MoveitActions/planning_group]");
      return;
    }

    // setup using just the name of the planning group you would like to control and plan for.
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(planning_group);
    visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("base_link");

    BT_ROS_INFO_STREAM("Available: target poses:");
    for (std::string name : move_group->getNamedTargets())
    {
      BT_ROS_INFO_STREAM(" [" << name << "]");
    }
  }

  ~MoveitMoveToFixConfiguration()
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

    std::string goal_name;
    auto res = BT::TreeNode::getInput("goal_name", goal_name);
    if (!res)
    {
      BT_ROS_ERROR_STREAM("Could not load goal_name: " << res.error());
      BT::TreeNode::setOutput("goal_name", std::string());
      return BT::NodeStatus::FAILURE;
    }

    BT_ROS_INFO_STREAM("Starting arm motion to pose named: [" << goal_name << "]");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
        move_group->getCurrentState()->getJointModelGroup(planning_group);

    // move_group->setMaxVelocityScalingFactor(0.1);
    move_group->setPlanningTime(planning_time);
    move_group->allowReplanning(allow_replanning);
    move_group->setGoalPositionTolerance(position_tolerance);
    move_group->setGoalOrientationTolerance(orientation_tolerance);
    move_group->setStartStateToCurrentState();

    BT_ROS_DEBUG_STREAM("Generating a motion plan to pose named: [" << goal_name << "]");

    // Finally plan and execute the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group->setNamedTarget(goal_name);

    moveit_msgs::MoveItErrorCodes s = move_group->plan(plan);
    if (s.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      BT_ROS_ERROR_STREAM("Fail to plan to goal name: " << goal_name << " error: " << getMoveitErrorCodeDescription(s));
      return BT::NodeStatus::FAILURE;
    }

    BT_ROS_DEBUG_STREAM("Executing motion plan to pose named: [" << goal_name << "]");

    moveit::core::MoveItErrorCode success = true;
    if (execute_trajectory)
    {
      move_group->stop();
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
      BT_ROS_ERROR_STREAM("Error move_group->execute(plan)");
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      ros::Duration(0.5).sleep();
      return BT::NodeStatus::SUCCESS;
    }
  }

  void halt() override
  {
    move_group->stop();
    setStatus(BT::NodeStatus::IDLE);
  }
};
}  // namespace MoveitActionsGroup
#endif  // MOVE_TO_NAMED_GOAL_H