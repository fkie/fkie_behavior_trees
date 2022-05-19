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

#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <numeric>
#include <ros/ros.h>
#include <vector>

/**
 * @brief Extract string content from a given [fileName]
 */
inline std::string readContentFile(const std::string& fileName)
{
  std::ifstream ifs(fileName.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

  std::ifstream::pos_type fileSize = ifs.tellg();
  ifs.seekg(0, std::ios::beg);

  std::vector<char> bytes(fileSize);
  ifs.read(bytes.data(), fileSize);

  return std::string(bytes.data(), fileSize);
}

/**
 * @brief Read a ROS parameter
 */
template <typename T>
bool getROSParameter(std::string name, T& param, bool print = true)
{
  ros::NodeHandle private_node = ros::NodeHandle("~");
  const T default_value = param;
  bool r = private_node.param<T>(name, param, default_value);

  if (print)
  {
    BT_ROS_INFO_STREAM(name << ": [" << param << "]");
  }

  return r;
}

/**
 * @brief Read ROS parameter (array-like)
 */
template <typename T>
bool getROSParameterVector(std::string name, T& param, bool print = true)
{
  ros::NodeHandle private_node = ros::NodeHandle("~");
  const T default_value = param;
  bool r = private_node.param<T>(name, param, default_value);

  if (print)
  {
    std::string s =
        std::accumulate(param.begin(), param.end(), std::string{},
                        [](std::string& s, const std::string& piece) -> decltype(auto) { return s += piece + ", "; });

    BT_ROS_INFO_STREAM(name << ": [" << s << "]");
  }

  return r;
}

#endif  // UTILS_H