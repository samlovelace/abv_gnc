#!/bin/bash

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
