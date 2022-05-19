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

#ifndef BEHAVIOR_TREE_PARAMETERS_H_
#define BEHAVIOR_TREE_PARAMETERS_H_

#include <fkie_behavior_tree_manager/logging.h>
#include <ros/ros.h>

class BehaviorTreeParameters
{
public:
  double rate;
  double seconds_wait_after_stop;
  bool start_tree_at_init;

  std::string tree_static_xml;
  std::string tree_static_file;

  // action plugins
  std::vector<std::string> action_plugins;

  // logging
  bool enable_global_log_publisher;
  std::string logger_type;

  bool RosMsgLogger_enabled;

  bool StdCoutLogger_enabled;

  bool PublisherZMQ_enabled;
  int PublisherZMQ_max_msg_per_second;
  int PublisherZMQ_publisher_port;
  int PublisherZMQ_server_port;

  bool FileLogger_enabled;
  std::string FileLogger_file_path;

  bool MinitraceLogger_enabled;
  std::string MinitraceLogger_file_path;

  BehaviorTreeParameters(){};

  void readParameters()
  {
    readParam("tree/rate", rate, 5.0);
    readParam("tree/start_tree_at_init", start_tree_at_init, true);
    readParam("tree/static_xml", tree_static_xml, std::string(""));
    readParam("tree/static_file", tree_static_file, std::string("tree.xml"));
    readParam("tree/seconds_wait_after_stop", seconds_wait_after_stop, 3.0);

    readParam("logging/enable_global_log_publisher", enable_global_log_publisher, true);
    readParam("logging/logger_type", logger_type, std::string(""));
    readParam("logging/RosMsgLogger/enabled", RosMsgLogger_enabled, false);
    readParam("logging/StdCoutLogger/enabled", StdCoutLogger_enabled, false);
    readParam("logging/PublisherZMQ/enabled", PublisherZMQ_enabled, false);
    readParam("logging/PublisherZMQ/max_msg_per_second", PublisherZMQ_max_msg_per_second, 20);
    readParam("logging/PublisherZMQ/publisher_port", PublisherZMQ_publisher_port, 1666);
    readParam("logging/PublisherZMQ/server_port", PublisherZMQ_server_port, 1667);
    readParam("logging/FileLogger/enabled", FileLogger_enabled, false);
    readParam("logging/FileLogger/file_path", FileLogger_file_path, std::string("bt_trace.fbl"));
    readParam("logging/MinitraceLogger/enabled", MinitraceLogger_enabled, false);
    readParam("logging/MinitraceLogger/file_path", MinitraceLogger_file_path, std::string("bt_trace.json"));

    readParamNoPrint("action_plugins", action_plugins, std::vector<std::string>());

    // validate parameters
    if (PublisherZMQ_enabled &&
        (PublisherZMQ_publisher_port <= 0 || PublisherZMQ_server_port <= 0 || PublisherZMQ_max_msg_per_second <= 0))
    {
      BT_ROS_WARN_STREAM(
          "PublisherZMQ enabled but has invalid config. Check parameters [logging/PublisherZMQ/...]. Disabling "
          "logger.");
      PublisherZMQ_enabled = false;
    }

    if (FileLogger_enabled && FileLogger_file_path.empty())
    {
      BT_ROS_WARN_STREAM(
          "FileLogger enabled but file_path is empty. Check parameter [logging/FileLogger/file_path]. Disabling "
          "logger.");
      FileLogger_enabled = false;
    }

    if (MinitraceLogger_enabled && MinitraceLogger_file_path.empty())
    {
      BT_ROS_WARN_STREAM(
          "MinitraceLogger enabled but file_path is empty. Check parameter [logging/MinitraceLogger/file_path]. "
          "Disabling logger.");
      MinitraceLogger_enabled = false;
    }

    if (!logger_type.empty())
    {
      BT_ROS_WARN_STREAM(
          "[logging/logger_type] parameter is deprecated. Check BT Wiki for more details about logging.");

      if (logger_type == "ZMQ_LOGGER")
      {
        BT_ROS_WARN_STREAM(
            "Overwriting [ZMQ_LOGGER] config because it was defined on [logging/logger_type] (deprecated).");
        PublisherZMQ_enabled = true;
      }
    }
  }

protected:
  ros::NodeHandle private_node = ros::NodeHandle("~");

  template <typename T>
  void readParam(const std::string& name, T& param, T default_value) const
  {
    readParamNoPrint(name, param, default_value);
    BT_ROS_INFO_STREAM(name << ": " << param);
  }

  template <typename T>
  void readParamNoPrint(const std::string& name, T& param, T default_value) const
  {
    private_node.param<T>(name, param, default_value);
  }
};

#endif /* BEHAVIOR_TREE_PARAMETERS_H_ */
