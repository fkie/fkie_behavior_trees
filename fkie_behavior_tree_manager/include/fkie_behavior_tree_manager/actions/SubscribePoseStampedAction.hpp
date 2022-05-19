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

#ifndef SUBSCRIBER_POSE_3D_ACTION_NODE_H
#define SUBSCRIBER_POSE_3D_ACTION_NODE_H

#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_listener.h>

#include <behaviortree_cpp_v3/decorator_node.h>
#include <fkie_behavior_tree_manager/logging.h>
#include <geometry_msgs/PoseStamped.h>

namespace BehaviorTreeNodes
{
class SubscribePoseStampedAction : public BT::SyncActionNode
{
public:
  std::string name;
  ros::NodeHandle public_node;
  std::string topic_pose_in;
  std::string output_frame;
  ros::Subscriber sub_pose;

  geometry_msgs::PoseStamped current_pose_callback;
  geometry_msgs::PoseStamped current_pose;

  tf::TransformListener tf_listener;

  bool valid_init = false;
  bool new_pose_received = false;

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = { BT::OutputPort<geometry_msgs::PoseStamped>("target_pose_3d") };
    return ports;
  }

  SubscribePoseStampedAction(const std::string& _name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(_name, config), name("[" + _name + "] ")
  {
    ros::NodeHandle private_node("~");

    std::string parameter_topic_pose_in = "SubscribePoseStampedAction/" + _name + "_topic_pose_in";
    std::string parameter_output_frame = "SubscribePoseStampedAction/" + _name + "_output_frame";

    private_node.param<std::string>(parameter_topic_pose_in, topic_pose_in, std::string(""));
    private_node.param<std::string>(parameter_output_frame, output_frame, std::string(""));

    if (topic_pose_in.empty())
    {
      BT_ROS_ERROR_STREAM(name << "topic_pose_in is empty. Check parameter [" << parameter_topic_pose_in << "]");
      return;
    }

    if (output_frame.empty())
    {
      BT_ROS_ERROR_STREAM(name << "output_frame is empty. Check parameter [" << parameter_output_frame << "]");
      return;
    }

    sub_pose = public_node.subscribe(topic_pose_in, 1, &SubscribePoseStampedAction::callbackPose, this);
    valid_init = true;

    BT_ROS_INFO_STREAM(name << " initialized");
  }

  inline BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);

    if (!valid_init)
      return BT::NodeStatus::FAILURE;

    if (new_pose_received)
    {
      new_pose_received = false;
      geometry_msgs::PoseStamped target_pose = current_pose_callback;
      target_pose.header.stamp = ros::Time::now();

      if (target_pose.header.frame_id == "")
      {
        BT_ROS_ERROR_STREAM(name << "target pose frame is empty");
        return BT::NodeStatus::FAILURE;
      }

      // transform point to desired target frame if needed
      if (output_frame != target_pose.header.frame_id)
      {
        // transform point
        tf::Stamped<tf::Pose> tf_pose;
        tf::StampedTransform transform;

        while (ros::ok() && !tf_listener.waitForTransform(output_frame, target_pose.header.frame_id, ros::Time(0),
                                                          ros::Duration(0.5)))
        {
          BT_ROS_INFO_STREAM(name << " Waiting for transform between [" << output_frame << "] and ["
                                  << target_pose.header.frame_id << "]");
          ros::Duration(0.2).sleep();
          ros::spinOnce();
        }
        tf_listener.lookupTransform(output_frame, target_pose.header.frame_id, ros::Time(0), transform);

        tf::poseStampedMsgToTF(target_pose, tf_pose);
        tf_pose.setData(transform * tf_pose);
        tf_pose.stamp_ = transform.stamp_;
        tf_pose.frame_id_ = transform.frame_id_;
        tf::poseStampedTFToMsg(tf_pose, target_pose);

        BT_ROS_DEBUG_STREAM(name << "transforming from [" << target_pose.header.frame_id << "] to [" << output_frame
                                 << "]");
        target_pose.header.frame_id = output_frame;
      }

      // save current pose to blackboard variable
      geometry_msgs::PoseStamped pose(target_pose);
      auto res = BT::TreeNode::setOutput("target_pose_3d", pose);
      if (!res)
      {
        BT_ROS_ERROR_STREAM(name << "Could not set output target_pose_3d: " << res.error());
        return BT::NodeStatus::FAILURE;
      }
      else
      {
        return BT::NodeStatus::SUCCESS;
      }
    }
    else
    {
      // reset current target pose
      BT::TreeNode::setOutput("target_pose_3d", geometry_msgs::PoseStamped());

      // ROS_WARN_STREAM("SubscribePoseStampedAction: Any new pose message was received, resetting target pose");
      return BT::NodeStatus::FAILURE;
    }
  }

  void callbackPose(const geometry_msgs::PoseStamped& msg)
  {
    if (!msg.header.frame_id.empty())
    {
      new_pose_received = true;
      current_pose_callback = msg;
      BT_ROS_DEBUG_STREAM(name << "New pose received in frame [" << msg.header.frame_id << "]");
    }
    else
      BT_ROS_DEBUG_STREAM(name << "New pose received with empty frame_id");
  }
};

}  // namespace BehaviorTreeNodes

#endif  // SUBSCRIBER_POSE_3D_ACTION_NODE_H