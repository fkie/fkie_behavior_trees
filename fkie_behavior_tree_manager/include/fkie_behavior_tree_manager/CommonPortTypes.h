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

#ifndef ROBOT_CUSTOM_MESSAGE_TYPES_H
#define ROBOT_CUSTOM_MESSAGE_TYPES_H

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

struct Pose2D
{
  double x, y, theta = 0.0;
  std::string frame_id;
  Pose2D() = default;
  Pose2D(double x, double y) : x(x), y(y){};
  Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta){};
  Pose2D(double x, double y, double theta, std::string frame_id) : x(x), y(y), theta(theta), frame_id(frame_id){};
  Pose2D(const geometry_msgs::PoseStamped pose)
  {
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    frame_id = pose.header.frame_id;
    double roll, pitch, yaw;
    tf::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                        pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if (!std::isnan(yaw))
      theta = yaw;
  }

  Pose2D(const geometry_msgs::PointStamped point)
  {
    x = point.point.x;
    y = point.point.y;
    frame_id = point.header.frame_id;
    theta = 0.0;
  }

  template <class T>
  Pose2D(const T point, const std::string _frame_id)
  {
    x = point.x;
    y = point.y;
    frame_id = _frame_id;
    theta = 0.0;
  }

  template <class T>
  Pose2D(const T point, double _theta, const std::string _frame_id)
  {
    x = point.x;
    y = point.y;
    frame_id = _frame_id;
    theta = _theta;
  }

  geometry_msgs::PoseStamped toPoseStamped() const
  {
    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = frame_id;
    p.pose.position.x = x;
    p.pose.position.y = y;

    if (!std::isnan(theta))
    {
      tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, theta);
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();
    }
    else
    {
      p.pose.orientation.w = 1.0;
    }

    return p;
  }

  std::string toString() const
  {
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(theta);
  }
};

// Define custom type Point3D to support geometry_msgs::PointStamped
struct Point3D
{
  double x, y, z;
  std::string frame_id;
  Point3D(){};
  Point3D(double x, double y, double z) : x(x), y(y), z(z){};
  Point3D(const geometry_msgs::PointStamped p)
  {
    x = p.point.x;
    y = p.point.y;
    z = p.point.z;
    frame_id = p.header.frame_id;
  }

  // Creates ROS message Pointstamped from custom type Point3D
  geometry_msgs::PointStamped toPointStamped() const
  {
    geometry_msgs::PointStamped p;
    p.header.frame_id = frame_id;
    p.point.x = x;
    p.point.y = y;
    p.point.z = z;
    return p;
  }

  // Convert the custom type Point3D to string
  std::string toString() const
  {
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
  }
};

typedef std::vector<std::string> ListString;
typedef std::vector<Point3D> Polygon3D;

namespace BT
{
template <>
inline Pose2D convertFromString(BT::StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3)
  {
    throw std::runtime_error("BT::convertFromString: invalid Pose2D");
  }
  else
  {
    Pose2D output(BT::convertFromString<double>(parts[0]), BT::convertFromString<double>(parts[1]),
                  BT::convertFromString<double>(parts[2]));

    return output;
  }
}

template <>
inline ListString convertFromString(BT::StringView key)
{
  auto parts = BT::splitString(key, ';');
  ListString output;
  output.reserve(parts.size());
  for (const BT::StringView& part : parts)
  {
    output.push_back(BT::convertFromString<std::string>(part));
  }
  return output;
}

// Begin: Point3D Template specialization to convert a string to Point3D
template <>
inline Point3D convertFromString(BT::StringView key)
{
  // three real numbers separated by semicolons for x,y,z values
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3)
  {
    ROS_FATAL_STREAM("BT::convertFromString: invalid Point3D, Point3D should have 3 values recieved "
                     << parts.size() << " values instead.");
    exit(EXIT_FAILURE);
  }
  else
  {
    Point3D output(BT::convertFromString<double>(parts[0]), BT::convertFromString<double>(parts[1]),
                   BT::convertFromString<double>(parts[2]));

    return output;
  }
}
// End: Point3D Template

// Begin: Polygon3D Template specialisation to convert a string to a Polygon3D
template <>
inline Polygon3D convertFromString(BT::StringView key)
{
  // three real numbers separated by semicolons for x,y,z values
  auto points = BT::splitString(key, ';');
  if (points.size() == 0 or points.size() < 3)
  {
    ROS_FATAL_STREAM("BT::convertFromString: Invalid Polygon3D, expects 3 or more values, recieved "
                     << points.size() << " values instead.");
    exit(EXIT_FAILURE);
  }
  else
  {
    Polygon3D output_vector;

    for (BT::StringView p : points)
    {
      auto p_vector = BT::splitString(p, ',');

      if (p_vector.size() > 2)
      {
        Point3D output(BT::convertFromString<double>(p_vector[0]), BT::convertFromString<double>(p_vector[1]),
                       BT::convertFromString<double>(p_vector[2]));

        output_vector.push_back(output);
      }
    }

    return output_vector;
  }
}
// End: Polygon3D Template
}  // namespace BT

#endif  // ROBOT_CUSTOM_MESSAGE_TYPES_H