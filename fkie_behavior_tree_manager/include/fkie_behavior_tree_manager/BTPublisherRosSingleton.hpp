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

#ifndef BT_PUBLISHER_ROS_SINGLETONE_H_
#define BT_PUBLISHER_ROS_SINGLETONE_H_

#include <algorithm>
#include <memory>
#include <mutex>
#include <thread>

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

#include <fkie_behavior_tree_msgs/BTLogging.h>

class BTPublisherRosSingleton
{
private:
  inline static BTPublisherRosSingleton* pinstance_{ nullptr };
  inline static std::mutex mutex_;

  ros::NodeHandle public_node;
  ros::Publisher pub;
  bool enable_publisher = false;

  std::mutex mutex_pub;

protected:
  BTPublisherRosSingleton()
  {
    enablePublisher();
  }
  ~BTPublisherRosSingleton()
  {
    disablePublisher();
  }

public:
  BTPublisherRosSingleton(BTPublisherRosSingleton& other) = delete;  // should not be cloneable.
  void operator=(const BTPublisherRosSingleton&) = delete;           // should not be assignable.

  static BTPublisherRosSingleton* GetInstance()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pinstance_ == nullptr)
    {
      pinstance_ = new BTPublisherRosSingleton();
    }
    return pinstance_;
  }

  /**
   * @brief Enable ROS publisher for fixed topic [bt_logging] from type [std_msgs::String]
   */
  void enablePublisher()
  {
    pub = public_node.advertise<fkie_behavior_tree_msgs::BTLogging>("bt_logging", 5);
    enable_publisher = true;
  }

  /**
   * @brief Enable ROS publisher for fixed topic [bt_logging] from type [std_msgs::String]
   */
  void disablePublisher()
  {
    pub.shutdown();
    enable_publisher = false;
  }

  /**
   * @brief Publish [text] to an [std_msgs::String] message in ROS
   */
  void publish(const ros::console::levels::Level level, const std::string& text)
  {
    if (!enable_publisher)
      return;

    if (pub.getNumSubscribers() == 0)
      return;

    // TODO: Check if required
    if (level == ros::console::levels::Level::Debug)
      return;

    {
      const std::lock_guard<std::mutex> lock(mutex_pub);
      fkie_behavior_tree_msgs::BTLogging msg;
      msg.header.stamp = ros::Time::now();
      msg.source = public_node.getNamespace();
      msg.level = levelToString(level);
      msg.data = text;
      pub.publish(msg);
    }
  }

  /**
   * @brief Publish [stream] to an [std_msgs::String] message in ROS
   */
  void publishStream(const ros::console::levels::Level level, const std::stringstream& stream)
  {
    publish(level, stream.str());
  }

  std::string levelToString(const ros::console::levels::Level& level) const
  {
    switch (level)
    {
      case ros::console::levels::Level::Debug:
        return "DEBUG";
        break;

      case ros::console::levels::Level::Info:
        return "INFO";
        break;

      case ros::console::levels::Level::Warn:
        return "WARN";
        break;

      case ros::console::levels::Level::Error:
        return "ERROR";
        break;

      case ros::console::levels::Level::Fatal:
        return "FATAL";
        break;

      default:
        return "UNKNOWN";
        break;
    }
  }
};

#endif /* BT_PUBLISHER_ROS_SINGLETONE_H_ */
