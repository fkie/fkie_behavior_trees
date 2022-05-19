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

#ifndef ROBOT_BEHAVIOR_TREE_MANAGER_H
#define ROBOT_BEHAVIOR_TREE_MANAGER_H

#include <ros/ros.h>
#include <string>
#include <topic_tools/shape_shifter.h>

// loggers
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

#include <fkie_behavior_tree_manager/CommonPortTypes.h>
#include <fkie_behavior_tree_manager/loggers/RosMsgLogger.hpp>
#include <fkie_behavior_tree_manager/logging.h>
#include <fkie_behavior_tree_manager/utils.hpp>

#include <fkie_behavior_tree_msgs/BTStatus.h>

#include "BehaviorTreeParameters.hpp"

class BehaviorTreeManager
{
public:
  BT::BehaviorTreeFactory bt_factory;
  std::unique_ptr<BT::Tree> tree;
  BehaviorTreeParameters params;

  bool is_tree_enabled = true;
  bool must_delete_tree = false;
  std::string file_tree_content;

  // logger definitions
  std::unique_ptr<BT::StdCoutLogger> std_cout_logger;
  std::unique_ptr<BT::FileLogger> file_logger;
  std::unique_ptr<BT::MinitraceLogger> minitrace_logger;
  std::unique_ptr<BT::PublisherZMQ> zmq_logger;
  std::unique_ptr<BehaviorTreeNodes::RosMsgLogger> ros_msg_logger;

  // control interface
  ros::Subscriber sub_start;
  ros::Subscriber sub_stop;
  ros::Publisher pub_bt_status;

private:
  BTPublisherRosSingleton* s_bt_publisher = BTPublisherRosSingleton::GetInstance();

  std::string current_static_tree;
  std::string current_path_file;

  std::mutex mutex_tree;

public:
  BehaviorTreeManager();
  ~BehaviorTreeManager();

  /**
   * @brief Callback requesting to start the execution of current Tree
   */
  void callbackStart(const topic_tools::ShapeShifter::ConstPtr& msg);

  /**
   * @brief Callback requesting to stop the execution of current Tree
   */
  void callbackStop(const topic_tools::ShapeShifter::ConstPtr& msg);

  /**
   * @brief Publish the current execution state of the tree
   */
  void reportBTState();

  /**
   * @brief Main loop, here will be called the root node of the tree continuously.
   */
  void spin();

  /**
   * @brief Use [registerFromPlugin] from BT Factory to register plugins from ROS parameter [action_plugins]
   */
  void registerActionPlugins();

  /**
   * @brief Initializes the tree using an XML text string
   */
  void initializeTreeFromText(const std::string static_tree);

  /**
   * @brief Initializes the tree using a file
   */
  void initializeTreeFromFile(const std::string path_file);

  /**
   * @brief Reinitialize the tree, using [current_static_tree] or [current_path_file]
   */
  bool reinitializeTree();

  /**
   * @brief Initializes the loggers according to ROS parameter [logging/...]
   */
  void initializeLoggers();

  /**
   * @brief Reset/Destroy all loggers
   */
  void resetLoggers();
};

#endif  // ROBOT_BEHAVIOR_TREE_MANAGER_H