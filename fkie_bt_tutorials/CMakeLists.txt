cmake_minimum_required(VERSION 3.0.2)
project(fkie_bt_tutorials)

find_package(catkin REQUIRED COMPONENTS
  fkie_behavior_tree_manager
)

find_package(BehaviorTreeV3)

catkin_package(
  CATKIN_DEPENDS fkie_behavior_tree_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

###########
## Build ##
###########

# static
add_library(TutorialActions  STATIC src/TutorialActions.cpp )
target_link_libraries(TutorialActions PRIVATE ${catkin_LIBRARIES} BT::behaviortree_cpp_v3)
set_target_properties(TutorialActions PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
target_compile_features(TutorialActions PRIVATE cxx_std_17)

# plugin
add_library(TutorialActions_dyn SHARED src/TutorialActions.cpp )
target_link_libraries(TutorialActions_dyn PRIVATE ${catkin_LIBRARIES} BT::behaviortree_cpp_v3)
target_compile_definitions(TutorialActions_dyn  PRIVATE BT_PLUGIN_EXPORT)
set_target_properties(TutorialActions_dyn PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
target_compile_features(TutorialActions_dyn PRIVATE cxx_std_17)

#############
## Install ##
#############

install(
  TARGETS TutorialActions TutorialActions_dyn
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)