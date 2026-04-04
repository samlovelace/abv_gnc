#!/bin/bash
##########################################################################
# Runs an ABV package binary with the required ROS2 environment sourced.
# Intended for use on Jetson hardware where packages must be launched
# directly (e.g. with sudo) rather than via ros2 run.
#
# Usage:
#   ./run.sh <package_name>
#
# Must be run from the workspace root.
##########################################################################

PKG_NAME=$1

# Check if argument is provided
if [ -z "$PKG_NAME" ]; then
  echo "Usage: run.sh <package_name>"
  exit 1
fi

# Source ROS and workspace setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Execute the binary
exec install/${PKG_NAME}/lib/${PKG_NAME}/${PKG_NAME} "${@:2}"
