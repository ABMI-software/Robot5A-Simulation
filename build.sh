#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
exec colcon build
