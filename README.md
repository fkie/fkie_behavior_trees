# FKIE Behavior Trees

This repository contains packages for executing Behavior Trees in ROS. It is powered by [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) and supports navigation and manipulation tasks using [Move Base Flex](http://wiki.ros.org/move_base_flex) and [MoveIt](https://moveit.ros.org/). It also provides RVIZ visualization and debugging tools.

![packbot_arm_nav](https://user-images.githubusercontent.com/748097/169791484-5d4ef6a2-efcc-4a56-afaa-941c340163cb.gif)

## Available Packages

- ```fkie_behavior_tree_manager``` interfaces with behavior tree engine
- ```fkie_bt_move_base_actions``` contains actions for interfacing with [Move Base Flex](http://wiki.ros.org/move_base_flex)
- ```fkie_bt_moveit_actions``` contains actions for interfacing with [MoveIt](https://moveit.ros.org/)
- ```fkie_bt_visualization``` contains RVIZ plugins for BT visualization
- ```fkie_bt_tutorials``` contains tutorials for creating new BT actions 

## Documentation

Check our [Wiki](https://github.com/fkie/fkie_behavior_trees/wiki) for tutorials and examples.

## How to Compile

- Install dependencies:

```
sudo apt install ros-noetic-behaviortree-cpp-v3 ros-noetic-mbf-msgs  qt5-default ros-noetic-moveit-ros ros-noetic-moveit-visual-tools
```

If you want to use all packages, you can compile the meta-package `fkie_behavior_trees`, using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

```
cd ros/src
git clone https://github.com/fkie/fkie_behavior_trees
cd fkie_behavior_trees/fkie_behavior_trees/
catkin build --this
```

## Try it out

- If you want to run a very simple example of Behavior Tree Manager:

```
roslaunch fkie_behavior_tree_manager behavior_tree.launch
```

## Authors

- Francisco J. Garcia R. [E-Mail](francisco.garcia.rosas@fkie.fraunhofer.de)
- Menaka Naazare - [E-Mail](menaka.naazare@fkie.fraunhofer.de)

## License

```
Copyright 2022 Fraunhofer FKIE

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```
