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

#ifndef BT_LOGGING_H_
#define BT_LOGGING_H_

#include "BTPublisherRosSingleton.hpp"
#include <algorithm>
#include <ostream>
#include <ros/ros.h>
#include <sstream>
#include <string>

inline void replaceAllOccurrencesString(std::string& str, const std::string& search, const std::string& replace)
{
  size_t pos = 0;
  while ((pos = str.find(search, pos)) != std::string::npos)
  {
    str.replace(pos, search.length(), replace);
    pos += replace.length();
  }
}

inline std::string __clearFunctionName(std::string str)
{
  str = str.substr(0, str.find("("));  // Remove parameters

  replaceAllOccurrencesString(str, "BT::NodeStatus ", "");

  std::string::size_type i = str.find(" ");
  if (i != std::string::npos)
    str.erase(0, i + 1);  // Remove return type

  replaceAllOccurrencesString(str, "::", ".");

  std::size_t found_point = str.rfind(".");
  if (found_point != std::string::npos)
    str = str.substr(0, found_point);  // Remove last dot: Function method

  return str;
}

#define _REMOVE_RETURN_TYPE_(str)                                                                                      \
  str.find(" ") != std::string::npos ? ({                                                                              \
    str.erase(str.find(" "), str.length());                                                                            \
    str;                                                                                                               \
  }) :                                                                                                                 \
                                       str
#define _REMOVE_PARAMETERS_(str) std::string(str).substr(0, std::string(str).find("("))
#define _FUNCTION_NAME_()                                                                                              \
  std::string(ROSCONSOLE_ROOT_LOGGER_NAME) + "." + __clearFunctionName(std::string(__PRETTY_FUNCTION__))

// Templates for ROS_XXXXX
template <typename T>
inline void BT_ROS_DEBUG(const T& args)
{
  BTPublisherRosSingleton::GetInstance()->publish(::ros::console::levels::Debug, args);
  ROS_LOG(::ros::console::levels::Debug, _FUNCTION_NAME_(), args);
}

template <typename T>
inline void BT_ROS_INFO(const T& args)
{
  BTPublisherRosSingleton::GetInstance()->publish(::ros::console::levels::Info, args);
  ROS_LOG(::ros::console::levels::Info, _FUNCTION_NAME_(), args);
}

template <typename T>
inline void BT_ROS_WARN(const T& args)
{
  BTPublisherRosSingleton::GetInstance()->publish(::ros::console::levels::Warn, args);
  ROS_LOG(::ros::console::levels::Warn, _FUNCTION_NAME_(), args);
}

template <typename T>
inline void BT_ROS_ERROR(const T& args)
{
  BTPublisherRosSingleton::GetInstance()->publish(::ros::console::levels::Error, args);
  ROS_LOG(::ros::console::levels::Error, _FUNCTION_NAME_(), args);
}

template <typename T>
inline void BT_ROS_FATAL(const T& args)
{
  BTPublisherRosSingleton::GetInstance()->publish(::ros::console::levels::Fatal, args);
  ROS_LOG(::ros::console::levels::Fatal, _FUNCTION_NAME_(), args);
}

// TODO: Try to remove macros (perhaps templates?)
// I did find a nicer way of implementing streams without those macros

#define BT_ROS_DEBUG_STREAM(msg)                                                                                       \
  {                                                                                                                    \
    std::stringstream __bt__ss__;                                                                                      \
    __bt__ss__ << msg;                                                                                                 \
    BTPublisherRosSingleton::GetInstance()->publishStream(::ros::console::levels::Debug, __bt__ss__);                  \
    ROS_LOG_STREAM(::ros::console::levels::Debug, _FUNCTION_NAME_(), msg);                                             \
  }

#define BT_ROS_INFO_STREAM(msg)                                                                                        \
  {                                                                                                                    \
    std::stringstream __bt__ss__;                                                                                      \
    __bt__ss__ << msg;                                                                                                 \
    BTPublisherRosSingleton::GetInstance()->publishStream(::ros::console::levels::Info, __bt__ss__);                   \
    ROS_LOG_STREAM(::ros::console::levels::Info, _FUNCTION_NAME_(), msg);                                              \
  }

#define BT_ROS_WARN_STREAM(msg)                                                                                        \
  {                                                                                                                    \
    std::stringstream __bt__ss__;                                                                                      \
    __bt__ss__ << msg;                                                                                                 \
    BTPublisherRosSingleton::GetInstance()->publishStream(::ros::console::levels::Warn, __bt__ss__);                   \
    ROS_LOG_STREAM(::ros::console::levels::Warn, _FUNCTION_NAME_(), msg);                                              \
  }

#define BT_ROS_ERROR_STREAM(msg)                                                                                       \
  {                                                                                                                    \
    std::stringstream __bt__ss__;                                                                                      \
    __bt__ss__ << msg;                                                                                                 \
    BTPublisherRosSingleton::GetInstance()->publishStream(::ros::console::levels::Error, __bt__ss__);                  \
    ROS_LOG_STREAM(::ros::console::levels::Error, _FUNCTION_NAME_(), msg);                                             \
  }

#define BT_ROS_FATAL_STREAM(msg)                                                                                       \
  {                                                                                                                    \
    std::stringstream __bt__ss__;                                                                                      \
    __bt__ss__ << msg;                                                                                                 \
    BTPublisherRosSingleton::GetInstance()->publishStream(::ros::console::levels::Fatal, __bt__ss__);                  \
    ROS_LOG_STREAM(::ros::console::levels::Fatal, _FUNCTION_NAME_(), msg);                                             \
  }

//#define BT_ROS_INFO(...) ROS_LOG(::ros::console::levels::Info, _FUNCTION_NAME_(), __VA_ARGS__)
//#define BT_ROS_DEBUG(...) ROS_LOG(::ros::console::levels::Debug, _FUNCTION_NAME_(), __VA_ARGS__)
//#define BT_ROS_WARN(...) ROS_LOG(::ros::console::levels::Warn, _FUNCTION_NAME_(), __VA_ARGS__)
//#define BT_ROS_ERROR(...) ROS_LOG(::ros::console::levels::Error, _FUNCTION_NAME_(), __VA_ARGS__)
//#define BT_ROS_FATAL(...) ROS_LOG(::ros::console::levels::Fatal, _FUNCTION_NAME_(), __VA_ARGS__)

//#define BT_ROS_INFO_STREAM(msg) ROS_LOG_STREAM(::ros::console::levels::Info, _FUNCTION_NAME_(), msg)
//#define BT_ROS_DEBUG_STREAM(msg) ROS_LOG_STREAM(::ros::console::levels::Debug, _FUNCTION_NAME_(), msg)
//#define BT_ROS_WARN_STREAM(msg) ROS_LOG_STREAM(::ros::console::levels::Warn, _FUNCTION_NAME_(), msg)
//#define BT_ROS_ERROR_STREAM(msg) ROS_LOG_STREAM(::ros::console::levels::Error, _FUNCTION_NAME_(), msg)
//#define BT_ROS_FATAL_STREAM(msg) ROS_LOG_STREAM(::ros::console::levels::Fatal, _FUNCTION_NAME_(), msg)

#endif /* BT_LOGGING_H_ */
