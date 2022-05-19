cmake_minimum_required(VERSION 3.0.2)
project(fkie_behavior_trees)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
