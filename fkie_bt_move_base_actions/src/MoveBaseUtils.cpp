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

#include "MoveBaseUtils.h"

namespace MoveBaseActionsGroup
{
BT::NodeStatus moveBaseHeartbeat(BT::TreeNode& self)
{
  std::string message;
  self.getInput("message", message);
  BT_ROS_INFO_STREAM("moveBaseHeartbeat: " << message);
  return BT::NodeStatus::SUCCESS;
};

BT::NodeStatus newTargetPoseReceived(BT::TreeNode& self)
{
  ros::NodeHandle private_node("~");
  std::string topic_target_pose, frame_id_target_pose, topic_exe_path;
  float timeout_target_pose = 2.0;
  private_node.param<std::string>("MoveBaseActions/topic_target_pose", topic_target_pose, std::string(""));
  private_node.param<std::string>("MoveBaseActions/frame_id_target_pose", frame_id_target_pose, std::string(""));
  private_node.param<float>("MoveBaseActions/timeout_target_pose", timeout_target_pose, 1.0f);

  // wait [timeout_target_pose] for first message
  geometry_msgs::PoseStampedConstPtr target_pose_once =
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>(topic_target_pose, ros::Duration(timeout_target_pose));
  if (!target_pose_once)
  {
    return BT::NodeStatus::FAILURE;
  }

  BT_ROS_WARN_STREAM("newTargetPoseReceived: new goal received!");

  geometry_msgs::PoseStamped target_pose = *target_pose_once;
  if (!target_pose.header.frame_id.empty() && !frame_id_target_pose.empty())
  {
    // transform point
    tf::Stamped<tf::Pose> tf_pose;
    tf::StampedTransform transform;

    tf::TransformListener tf_listener;
    while (ros::ok() && !tf_listener.waitForTransform(frame_id_target_pose, target_pose.header.frame_id, ros::Time(0),
                                                      ros::Duration(1.5)))
    {
      BT_ROS_INFO_STREAM("newTargetPoseReceived: Waiting for transform between ["
                         << frame_id_target_pose << "] and [" << target_pose.header.frame_id << "]");
      ros::Duration(0.2).sleep();
      ros::spinOnce();
    }
    tf_listener.lookupTransform(frame_id_target_pose, target_pose.header.frame_id, ros::Time(0), transform);

    tf::poseStampedMsgToTF(target_pose, tf_pose);
    tf_pose.setData(transform * tf_pose);
    tf_pose.stamp_ = transform.stamp_;
    tf_pose.frame_id_ = transform.frame_id_;
    tf::poseStampedTFToMsg(tf_pose, target_pose);

    BT_ROS_INFO_STREAM("newTargetPoseReceived: transforming from [" << target_pose.header.frame_id << "] to ["
                                                                    << frame_id_target_pose << "]");
    target_pose.header.frame_id = frame_id_target_pose;
  }
  else if (target_pose.header.frame_id.empty() && frame_id_target_pose.empty())
  {
    BT_ROS_ERROR("newTargetPoseReceived: Missing param [frame_id_target_pose]");
    return BT::NodeStatus::FAILURE;
  }

  // stopping move base
  private_node.param("MoveBaseActions/topic_exe_path", topic_exe_path, std::string("exe_path"));
  std::unique_ptr<ClientUtilsExePathAction> ac;

  ac = std::make_unique<ClientUtilsExePathAction>(topic_exe_path, true);
  while (ros::ok() && !ac->waitForServer(ros::Duration(0.1)))
    ros::spinOnce();

  ac->cancelAllGoals();

  // target_pose.header.frame_id = frame_id_target_pose;
  Pose2D pose(target_pose);
  auto res = self.setOutput("target_pose", pose);
  if (!res)
  {
    BT_ROS_ERROR_STREAM("newTargetPoseReceived: Could not set output target_pose: " << res.error());
    return BT::NodeStatus::FAILURE;
  }

  BT_ROS_INFO_STREAM("Setting output: [target_pose] with frame_id: [" << target_pose.header.frame_id << "]");

  // return failure to restart move base loop
  return BT::NodeStatus::SUCCESS;
};

BT::NodeStatus CancelCurrentPoseAction(BT::TreeNode& self)
{
  ros::NodeHandle private_node("~");
  std::string topic_exe_path;

  BT_ROS_INFO_STREAM("CancelCurrentPoseAction: Cancelling current goals.");

  // stopping move base
  private_node.param("MoveBaseActions/topic_exe_path", topic_exe_path, std::string("exe_path"));
  std::unique_ptr<ClientUtilsExePathAction> ac;
  ac = std::make_unique<ClientUtilsExePathAction>(topic_exe_path, true);
  while (ros::ok() && !ac->waitForServer(ros::Duration(0.1)))
  {
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  }

  ac->cancelAllGoals();
  self.setOutput("target_pose", Pose2D());

  // TODO: Publish pose reset command!
  return BT::NodeStatus::SUCCESS;
};

BT::NodeStatus hasValidGoal(BT::TreeNode& self)
{
  Pose2D target_pose;
  auto res = self.getInput("target_pose", target_pose);
  if (!res)
  {
    self.setOutput("target_pose", Pose2D());
    BT_ROS_ERROR_STREAM("hasValidGoal: " << res.error());
    return BT::NodeStatus::FAILURE;
  }

  if (target_pose.frame_id.empty())
  {
    // ROS_INFO("hasValidGoal: invalid goal");
    return BT::NodeStatus::FAILURE;
  }

  // convert frame _id if necessary
  ros::NodeHandle private_node("~");
  std::string frame_id_target_pose;
  private_node.param<std::string>("MoveBaseActions/frame_id_target_pose", frame_id_target_pose, std::string(""));

  if (target_pose.frame_id != frame_id_target_pose)
  {
    geometry_msgs::PoseStamped target_pose_geo = target_pose.toPoseStamped();
    BT_ROS_INFO_STREAM("hasValidGoal: transforming from [" << target_pose_geo.header.frame_id << "] to ["
                                                           << frame_id_target_pose << "]");

    // transform point
    tf::Stamped<tf::Pose> tf_pose;
    tf::StampedTransform transform;

    tf::TransformListener tf_listener;
    while (ros::ok() && !tf_listener.waitForTransform(frame_id_target_pose, target_pose_geo.header.frame_id,
                                                      ros::Time(0), ros::Duration(1.5)))
    {
      BT_ROS_INFO_STREAM("hasValidGoal: Waiting for transform between [" << frame_id_target_pose << "] and ["
                                                                         << target_pose_geo.header.frame_id << "]");
      ros::Duration(0.2).sleep();
      ros::spinOnce();
    }
    tf_listener.lookupTransform(frame_id_target_pose, target_pose_geo.header.frame_id, ros::Time(0), transform);

    tf::poseStampedMsgToTF(target_pose_geo, tf_pose);
    tf_pose.setData(transform * tf_pose);
    tf_pose.stamp_ = transform.stamp_;
    tf_pose.frame_id_ = transform.frame_id_;
    tf::poseStampedTFToMsg(tf_pose, target_pose_geo);

    target_pose_geo.header.frame_id = frame_id_target_pose;
    target_pose = Pose2D(target_pose_geo);

    self.setOutput("target_pose", target_pose);
  }

  // return failure to restart move base loop
  // ROS_INFO_STREAM("hasValidGoal: Valid goal found: " << target_pose.frame_id);
  return BT::NodeStatus::SUCCESS;
};

}  // namespace MoveBaseActionsGroup