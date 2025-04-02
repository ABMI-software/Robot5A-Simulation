#!/bin/bash

set -e

source install/setup.bash
export GAZEBO_RESOURCE_PATH="$PWD/src/robot_description:/usr/share/gazebo-11/models:/usr/share/gazebo-11"
exec ros2 launch robot_control visual_sim.launch.py num_cameras:=2
