#!/bin/bash

set -e

source install/setup.bash
export GAZEBO_RESOURCE_PATH="$PWD/install/robot_description/share/robot_description:/usr/share/gazebo-11"
exec ros2 launch robot_control visual_sim.launch.py
