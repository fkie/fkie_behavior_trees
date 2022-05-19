cmake_minimum_required(VERSION 3.0.2)
project(fkie_bt_move_base_actions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mbf_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  fkie_behavior_tree_manager
  tf
  message_generation
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

add_message_files(
  FILES
  NavigationResult.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    mbf_msgs 
    actionlib_msgs 
    actionlib 
    geometry_msgs 
    nav_msgs
    fkie_behavior_tree_manager
    tf
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

###########
## Build ##
###########

# static
add_library(MoveBaseActions STATIC 
  src/MoveBaseActions.cpp
  src/MoveBaseUtils.cpp
  src/GetPath.cpp
  src/ExePathAsync.cpp
  src/ExePathSync.cpp
  src/CancelExePathSync.cpp
  src/Recovery.cpp
  src/NotifyNavigationResult.cpp
  src/NavigateToGoal.cpp
  src/NavigateToGoalSync.cpp
)
target_link_libraries(MoveBaseActions PRIVATE ${catkin_LIBRARIES} BT::behaviortree_cpp_v3)
set_target_properties(MoveBaseActions PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
add_dependencies(MoveBaseActions
  ${${PROJECT_NAME}_generate_messages_cpp}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(MoveBaseActions PRIVATE cxx_std_17)

# plugin
add_library(MoveBaseActions_dyn SHARED 
  src/MoveBaseActions.cpp
  src/MoveBaseUtils.cpp
  src/GetPath.cpp
  src/ExePathAsync.cpp
  src/ExePathSync.cpp
  src/CancelExePathSync.cpp
  src/Recovery.cpp
  src/NotifyNavigationResult.cpp
  src/NavigateToGoal.cpp
  src/NavigateToGoalSync.cpp
)
target_link_libraries(MoveBaseActions_dyn PRIVATE ${catkin_LIBRARIES} BT::behaviortree_cpp_v3)
target_compile_definitions(MoveBaseActions_dyn  PRIVATE BT_PLUGIN_EXPORT)
set_target_properties(MoveBaseActions_dyn PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
add_dependencies(MoveBaseActions_dyn
  ${${PROJECT_NAME}_generate_messages_cpp}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(MoveBaseActions_dyn PRIVATE cxx_std_17)


#############
## Install ##
#############

install(
  TARGETS MoveBaseActions MoveBaseActions_dyn
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
