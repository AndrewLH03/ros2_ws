#!/bin/bash
# Build and run all nodes for the CR3 Hand Tracking Dashboard System

# Exit on error
set -e

# Source ROS 2 installation first
source /opt/ros/jazzy/setup.bash

# Move to workspace root
cd ~/VSCode/ros2_ws

# Build the workspace
echo "Building workspace..."
colcon build

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "Starting all nodes for hand tracking dashboard system..."
echo "Press Ctrl+C to stop all nodes"

# Start camera interface node in background
echo "Starting camera node..."
ros2 run camera_interface camera_node &
CAMERA_PID=$!

# Wait a moment for camera to initialize
sleep 2

# Start perception nodes in background
echo "Starting hand pose node..."
ros2 run perception hand_pose_node &
HAND_POSE_PID=$!

echo "Starting body pose node..."
ros2 run perception body_pose_node &
BODY_POSE_PID=$!

echo "Starting coordinate transform node..."
ros2 run perception coordinate_transform_node &
COORD_TRANSFORM_PID=$!

# Wait a moment for perception to initialize
sleep 2

# Start control nodes in background
echo "Starting pose to command node..."
ros2 run control pose_to_command_node &
POSE_COMMAND_PID=$!

# Start UI nodes in background
echo "Starting mode switcher node..."
ros2 run ui mode_switcher_node &
MODE_SWITCHER_PID=$!

# Start the main UI dashboard (this should be the foreground process to see the OpenCV window)
echo "Starting UI dashboard (OpenCV window will appear)..."
ros2 run ui ui_dashboard_node

# Cleanup function to kill all background processes
cleanup() {
    echo ""
    echo "Stopping all nodes..."
    kill $CAMERA_PID $HAND_POSE_PID $BODY_POSE_PID $COORD_TRANSFORM_PID $POSE_COMMAND_PID $MODE_SWITCHER_PID 2>/dev/null || true
    echo "All nodes stopped."
}

# Set up trap to cleanup on script exit
trap cleanup EXIT
