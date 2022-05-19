# Fkie Behavior Trees

## Description

This repository contains packages for executing behavior trees in ROS. It supports navigation and manipulation tasks using [Move Base Flex](http://wiki.ros.org/move_base_flex) and [MoveIt](https://moveit.ros.org/) and also provides RVIZ visualization and debugging tools.

## Package Description

- ```fkie_behavior_tree_manager``` interfaces with behavior tree engine
- ```fkie_bt_move_base_actions``` contains actions for interfacing with [Move Base Flex](http://wiki.ros.org/move_base_flex)
- ```fkie_bt_moveit_actions``` contains actions for interfacing with [MoveIt](https://moveit.ros.org/)
- ```fkie_bt_visualization``` contains RVIZ plugins for BT visualization
- ```fkie_bt_tutorials``` contains tutorials for creating new BT actions 

## How to Compile it

### Install dependencies

First you should install dependencies:

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

## How to Test it

- If you want to run a very simple example of Behavior Tree Manager:

```
roslaunch fkie_behavior_tree_manager behavior_tree.launch
```

## Contributors

```
Francisco J. Garcia R.
francisco.garcia.rosas@fkie.fraunhofer.de

Menaka Naazare
menaka.naazare@fkie.fraunhofer.de
```
