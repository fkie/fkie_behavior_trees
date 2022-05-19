cmake_minimum_required(VERSION 3.0.2)
project(fkie_behavior_tree_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation message_runtime)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    BTNodeStatus.msg
    BTStatus.msg
    BTLogging.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
