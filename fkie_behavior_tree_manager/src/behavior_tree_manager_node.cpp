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

#include "BehaviorTreeManager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "behavior_tree_manager");

  // initializes ROS NodeHandles - Please do not remove!
  ros::NodeHandle public_node = ros::NodeHandle("");
  ros::NodeHandle private_node = ros::NodeHandle("~");

  BehaviorTreeManager bt_manager;

  // Update ros parameters
  bt_manager.params.readParameters();

  // register available action plugins
  BT_ROS_INFO_STREAM("Registered plugins:");
  for (std::string ap : bt_manager.params.action_plugins)
    BT_ROS_INFO_STREAM("  - " << ap);

  bt_manager.registerActionPlugins();
  bt_manager.reportBTState();

  // initialize static tree
  if (!bt_manager.params.tree_static_xml.empty())
  {
    bt_manager.initializeTreeFromText(bt_manager.params.tree_static_xml);
  }
  else if (!bt_manager.params.tree_static_file.empty())
  {
    bt_manager.initializeTreeFromFile(bt_manager.params.tree_static_file);
  }
  else
  {
    BT_ROS_ERROR("No tree is available. Check parameters [tree/static_xml] or [tree/static_file]");
    return 0;
  }

  // initialize loggin system
  bt_manager.initializeLoggers();

  bt_manager.spin();

  return 0;
}