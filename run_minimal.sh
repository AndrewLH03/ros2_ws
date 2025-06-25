#!/bin/bash
# Build and run the minimal ROS2 system for visualization

# Exit on error
set -e

# Move to workspace root
cd ~/VSCode/ros2_ws

# Build the workspace with symlink install
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Launch the minimal system
ros2 launch src/launch/minimal_bringup.launch.py
