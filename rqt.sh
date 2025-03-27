#!/bin/bash

set -e

# source the setup files
# https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files
for i in /opt/ros/humble/setup.bash install/setup.bash; do
  if [ -f "$i" ]; then
    . "$i"
  fi
done
exec rqt "$@"
