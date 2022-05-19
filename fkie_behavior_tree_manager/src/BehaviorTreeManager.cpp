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

#include "BehaviorTreeManager.h"

BehaviorTreeManager::BehaviorTreeManager()
{
  ros::NodeHandle nh("~");

  pub_bt_status = nh.advertise<fkie_behavior_tree_msgs::BTStatus>("bt_status", 5, false);

  // generic subscriber for starting the tree
  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback_start;
  callback_start = [&](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { callbackStart(msg); };
  sub_start = nh.subscribe("start_tree", 1, callback_start);

  // generic subscriber for stopping the tree
  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback_stop;
  callback_stop = [&](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { callbackStop(msg); };
  sub_stop = nh.subscribe("stop_tree", 1, callback_stop);

  if (params.enable_global_log_publisher)
    s_bt_publisher->enablePublisher();
};

BehaviorTreeManager::~BehaviorTreeManager()
{
  sub_start.shutdown();
  sub_stop.shutdown();
};

void BehaviorTreeManager::callbackStart(const topic_tools::ShapeShifter::ConstPtr& msg)
{
  BT_ROS_INFO("New request to start the tree execution");

  if (is_tree_enabled)
  {
    BT_ROS_INFO("Tree is enabled, ignoring request");
    return;
  }

  {
    std::lock_guard<std::mutex> lck(mutex_tree);

    // reinitialize the tree and loggers
    reinitializeTree();
    initializeLoggers();
    BT_ROS_WARN_STREAM("[" << ros::this_node::getName() << "] has been started!");
  }

  reportBTState();

  is_tree_enabled = true;
}

void BehaviorTreeManager::callbackStop(const topic_tools::ShapeShifter::ConstPtr& msg)
{
  BT_ROS_INFO("New request to stop the tree execution");

  if (!is_tree_enabled)
  {
    BT_ROS_INFO("Tree is disable, ignoring request");
    return;
  }

  // stop BT nodes
  tree->haltTree();

  // disable loggers
  if (std_cout_logger)
    std_cout_logger->setEnabled(false);
  if (file_logger)
    file_logger->setEnabled(false);
  if (minitrace_logger)
    minitrace_logger->setEnabled(false);
  if (zmq_logger)
    zmq_logger->setEnabled(false);
  if (ros_msg_logger)
    ros_msg_logger->setEnabled(false);

  // wait for the nodes to terminate (useful for async nodes)
  ros::Duration(params.seconds_wait_after_stop).sleep();

  must_delete_tree = true;
  is_tree_enabled = false;

  reportBTState();
}

void BehaviorTreeManager::reportBTState()
{
  fkie_behavior_tree_msgs::BTStatus bt_status_msg;
  bt_status_msg.tree_file = params.tree_static_file;

  // read file (only once) and send XML content
  if (file_tree_content.empty())
    file_tree_content = readContentFile(params.tree_static_file);

  bt_status_msg.tree_xml = file_tree_content;

  bt_status_msg.header.stamp = ros::Time::now();
  bt_status_msg.tree_is_running = is_tree_enabled;
  pub_bt_status.publish(bt_status_msg);
}

void BehaviorTreeManager::spin()
{
  // check if tree must start automatically
  is_tree_enabled = params.start_tree_at_init;

  ros::Rate r(params.rate);

  while (ros::ok())
  {
    reportBTState();

    if (tree)
    {
      if (is_tree_enabled)
      {
        BT_ROS_DEBUG_STREAM("Starting execution of [root] node on tree\n");
        tree->tickRoot();
      }
      else
      {
        BT_ROS_DEBUG_STREAM("Tree execution is disabled \n");

        // recreate the tree and loggers
        if (must_delete_tree)
        {
          resetLoggers();

          {
            std::lock_guard<std::mutex> lck(mutex_tree);
            tree.reset(nullptr);
          }

          BT_ROS_WARN_STREAM("[" << ros::this_node::getName() << "] has been stopped!");
          must_delete_tree = false;
        }
      }
    }

    reportBTState();
    ros::spinOnce();
    r.sleep();
  }
}

void BehaviorTreeManager::registerActionPlugins()
{
  // Register common conditions
  bt_factory.registerFromPlugin("libBehaviorTreeNodes_dyn.so");

  for (std::string ap : params.action_plugins)
  {
    BT_ROS_DEBUG_STREAM("Loading pluging: " << ap);
    bt_factory.registerFromPlugin(ap);
  }
}

void BehaviorTreeManager::initializeTreeFromText(const std::string static_tree)
{
  BT_ROS_INFO_STREAM("Initializing tree from text");
  tree.reset(new BT::Tree(bt_factory.createTreeFromText(static_tree)));
  current_static_tree = static_tree;
}

void BehaviorTreeManager::initializeTreeFromFile(const std::string path_file)
{
  BT_ROS_INFO_STREAM("Initializing tree from file");
  tree.reset(new BT::Tree(bt_factory.createTreeFromFile(path_file)));
  current_path_file = path_file;
}

bool BehaviorTreeManager::reinitializeTree()
{
  // create a new tree instance
  if (!tree)
  {
    tree.reset(new BT::Tree());
  }

  if (!current_static_tree.empty())
  {
    initializeTreeFromText(current_static_tree);
  }
  else if (!current_path_file.empty())
  {
    initializeTreeFromFile(current_path_file);
  }
  return false;
}

void BehaviorTreeManager::initializeLoggers()
{
  if (!tree)
  {
    BT_ROS_ERROR_STREAM("Tree is not initialized");
    return;
  }

  if (params.StdCoutLogger_enabled)
  {
    BT_ROS_INFO_STREAM("Creating StdCoutLogger");
    std_cout_logger.reset(new BT::StdCoutLogger(*tree));
  }

  if (params.FileLogger_enabled)
  {
    BT_ROS_INFO_STREAM("Creating FileLogger on file: " << params.FileLogger_file_path);
    file_logger.reset(new BT::FileLogger(*tree, params.FileLogger_file_path.c_str()));
  }

  if (params.MinitraceLogger_enabled)
  {
    BT_ROS_INFO_STREAM("Creating MinitraceLogger on file: " << params.MinitraceLogger_file_path);
    minitrace_logger.reset(new BT::MinitraceLogger(*tree, params.MinitraceLogger_file_path.c_str()));
  }

  if (params.PublisherZMQ_enabled)
  {
    BT_ROS_INFO_STREAM("Creating PublisherZMQ in ports: [publisher: "
                       << params.PublisherZMQ_publisher_port << " server: " << params.PublisherZMQ_server_port << "]");
    zmq_logger.reset(nullptr);
    zmq_logger = NULL;
    zmq_logger.reset(new BT::PublisherZMQ(*tree, params.PublisherZMQ_max_msg_per_second,
                                          params.PublisherZMQ_publisher_port, params.PublisherZMQ_server_port));
  }

  if (params.RosMsgLogger_enabled)
  {
    BT_ROS_INFO_STREAM("Creating RosMsgLogger");
    ros_msg_logger.reset(new BehaviorTreeNodes::RosMsgLogger(*tree, "bt_node_status"));
  }
}

void BehaviorTreeManager::resetLoggers()
{
  if (std_cout_logger)
  {
    std_cout_logger->setEnabled(false);
    std_cout_logger.reset(nullptr);
  }

  if (file_logger)
  {
    file_logger->setEnabled(false);
    file_logger.reset(nullptr);
  }

  if (minitrace_logger)
  {
    minitrace_logger->setEnabled(false);
    minitrace_logger.reset(nullptr);
  }

  if (zmq_logger)
  {
    zmq_logger->setEnabled(false);
    zmq_logger.reset(nullptr);
  }

  if (ros_msg_logger)
  {
    ros_msg_logger->setEnabled(false);
    ros_msg_logger.reset(nullptr);
  }
}
