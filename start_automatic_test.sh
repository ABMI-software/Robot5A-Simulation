#!/bin/bash

set -e

source install/setup.bash
export GAZEBO_RESOURCE_PATH="$PWD/src/robot_description:/usr/share/gazebo-11/models:/usr/share/gazebo-11"
ros2 run robot_data_process joint_sync_moveit_node
