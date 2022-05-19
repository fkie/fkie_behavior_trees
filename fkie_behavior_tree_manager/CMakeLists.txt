cmake_minimum_required(VERSION 3.0.2)
project(fkie_behavior_tree_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  topic_tools
  tf2_ros
  fkie_behavior_tree_msgs
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


set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs tf topic_tools fkie_behavior_tree_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${ZeroMQ_INCLUDE_DIR}
)

###########
## Build ##
###########

# static library
add_library(BehaviorTreeNodes STATIC src/BehaviorTreeNodes.cpp )
target_link_libraries(BehaviorTreeNodes PRIVATE ${catkin_LIBRARIES} BT::behaviortree_cpp_v3)
set_target_properties(BehaviorTreeNodes PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
target_compile_features(BehaviorTreeNodes PRIVATE cxx_std_17)
set_target_properties(BehaviorTreeNodes PROPERTIES  
  CXX_STANDARD 17  
  CXX_EXTENSIONS OFF)

# plugin
add_library(BehaviorTreeNodes_dyn SHARED src/BehaviorTreeNodes.cpp )
target_link_libraries(BehaviorTreeNodes_dyn PRIVATE ${catkin_LIBRARIES} BT::behaviortree_cpp_v3)
target_compile_definitions(BehaviorTreeNodes_dyn  PRIVATE BT_PLUGIN_EXPORT)
set_target_properties(BehaviorTreeNodes_dyn PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
target_compile_features(BehaviorTreeNodes_dyn PRIVATE cxx_std_17)
set_target_properties(BehaviorTreeNodes_dyn PROPERTIES  
  CXX_STANDARD 17  
  CXX_EXTENSIONS OFF)

# Node
add_executable(behavior_tree_manager_node
  src/behavior_tree_manager_node.cpp
  src/BehaviorTreeManager.cpp
)
add_dependencies(behavior_tree_manager_node 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_generate_messages_cpp}
  ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(behavior_tree_manager_node ${catkin_LIBRARIES} ${ZeroMQ_LIBRARY} BT::behaviortree_cpp_v3)
target_compile_features(behavior_tree_manager_node PRIVATE cxx_std_17)
set_target_properties(behavior_tree_manager_node PROPERTIES  
  CXX_STANDARD 17  
  CXX_EXTENSIONS OFF)


#############
## Install ##
#############

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Install all the include files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})    

# Mark executables and/or libraries for installation
install(TARGETS 
    BehaviorTreeNodes
    BehaviorTreeNodes_dyn
    behavior_tree_manager_node
  EXPORT BehaviorTreeNodes
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
