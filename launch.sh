#!/bin/bash

set -e

source install/setup.bash
exec ros2 launch robot_control visual_sim.launch.py
