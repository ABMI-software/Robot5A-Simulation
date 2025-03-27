#!/bin/bash

set -e

# source the setup files
# https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files
# https://classic.gazebosim.org/tutorials?tut=ros2_installing#Introduction
for i in /opt/ros/humble/setup.bash /usr/share/gazebo/setup.bash install/setup.bash; do
  if [ -f "$i" ]; then
    . "$i"
  fi
done
exec ros2 launch robot_control visual_sim.launch.py
