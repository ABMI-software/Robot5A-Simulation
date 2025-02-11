#!/bin/bash

set -e

# source the setup files
# https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files
source /opt/ros/jazzy/setup.bash
exec colcon build
