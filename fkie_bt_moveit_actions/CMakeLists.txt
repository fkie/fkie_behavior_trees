cmake_minimum_required(VERSION 3.0.2)
project(fkie_bt_moveit_actions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  fkie_behavior_tree_manager
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  eigen_conversions 
  tf_conversions
  pluginlib
  geometric_shapes
  tf
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# Look for system package
find_package(BehaviorTreeV3)
if(NOT TARGET BT::behaviortree_cpp_v3)
  # If not found, look for package in ROS workspace
  find_package(behaviortree_cpp_v3 REQUIRED)
  #catkin_lint: ignore_once order_violation
  add_library(BT::behaviortree_cpp_v3 INTERFACE IMPORTED)
  set_target_properties(BT::behaviortree_cpp_v3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${behaviortree_cpp_v3_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${behaviortree_cpp_v3_LIBRARIES}"
  )
endif()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS 
    roscpp rospy std_msgs actionlib_msgs actionlib
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    tf
    tf2_geometry_msgs
    fkie_behavior_tree_manager
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${Eigen_INCLUDE_DIRS}
)

###########
## Build ##
###########

# static
add_library(MoveitActions  STATIC src/MoveitActions.cpp )
target_link_libraries(MoveitActions PRIVATE ${catkin_LIBRARIES} ${Eigen_LIBRARIES} BT::behaviortree_cpp_v3)
set_target_properties(MoveitActions PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
target_compile_features(MoveitActions PRIVATE cxx_std_17)

# plugin
add_library(MoveitActions_dyn SHARED src/MoveitActions.cpp )
target_link_libraries(MoveitActions_dyn PRIVATE ${catkin_LIBRARIES} ${Eigen_LIBRARIES} BT::behaviortree_cpp_v3)
target_compile_definitions(MoveitActions_dyn  PRIVATE BT_PLUGIN_EXPORT)
set_target_properties(MoveitActions_dyn PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
target_compile_features(MoveitActions_dyn PRIVATE cxx_std_17)

#############
## Install ##
#############

install(
  TARGETS MoveitActions MoveitActions_dyn
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
