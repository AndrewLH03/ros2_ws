# CR3 Hand Tracking System - Build and Test Guide

## Overview

This system uses individual node testing approach rather than launch files. All testing is performed by building packages with `colcon build` and running nodes individually for precise control and debugging.

## Build Instructions

### Full System Build
```bash
cd /home/andrewlh/VSCode/ros2_ws
colcon build
source install/setup.bash
```

### Individual Package Build
```bash
# Build only perception package
colcon build --packages-select perception

# Build only control package  
colcon build --packages-select control

# Build only ui package
colcon build --packages-select ui

# Build only camera_interface package
colcon build --packages-select camera_interface
```

### Clean Build (if needed)
```bash
rm -rf build/ install/ log/
colcon build
```

## Node Testing Instructions

### 1. Testing Perception Nodes

**Prerequisites**: Ensure camera is connected and accessible

```bash
# Terminal 1: Camera feed
ros2 run camera_interface camera_node

# Terminal 2: Hand pose detection  
ros2 run perception hand_pose_node

# Terminal 3: Body pose detection
ros2 run perception body_pose_node

# Terminal 4: Coordinate transformation (requires hand_pose and body_pose)
ros2 run perception coordinate_transform_node

# Terminal 5: Pose filtering (optional)
ros2 run perception pose_filter_node
```

### 2. Testing Control Nodes

**Prerequisites**: Perception nodes must be running

```bash
# Terminal 1: Enhanced pose-to-command node
ros2 run control pose_to_command_node

# Terminal 2: Motion planner (optional)
ros2 run control motion_planner_node

# Terminal 3: Trajectory executor (optional)
ros2 run control trajectory_executor_node

# Terminal 4: Teleop backup control (optional)
ros2 run control teleop_node
```

### 3. Testing UI Nodes

**Prerequisites**: Camera and perception nodes running

```bash
# Terminal 1: Mode switcher
ros2 run ui mode_switcher_node

# Terminal 2: Enhanced dashboard (requires OpenCV display)
ros2 run ui ui_dashboard_node
```

## Topic Monitoring and Debugging

### Monitor All Topics
```bash
# List all active topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /perception/hand_pose
ros2 topic echo /perception/tracking_vector
ros2 topic echo /mode
ros2 topic echo /cr3/target_pose
```

### Check Topic Data Types
```bash
ros2 topic info /perception/hand_pose_robot_frame
ros2 topic info /perception/tracking_vector
ros2 topic info /ui/control_commands
```

### Node Information
```bash
# List all running nodes
ros2 node list

# Get node information
ros2 node info /coordinate_transform_node
ros2 node info /ui_dashboard_node
```

## Testing Sequences

### Minimal Hand Tracking Test
```bash
# 1. Build system
colcon build --packages-select camera_interface perception

# 2. Source environment
source install/setup.bash

# 3. Start basic pipeline
ros2 run camera_interface camera_node &
ros2 run perception hand_pose_node &
ros2 run perception coordinate_transform_node

# 4. Monitor output
ros2 topic echo /perception/wrist_pose
```

### Full Dashboard Test
```bash
# 1. Build all packages
colcon build

# 2. Source environment  
source install/setup.bash

# 3. Start perception pipeline
ros2 run camera_interface camera_node &
ros2 run perception hand_pose_node &
ros2 run perception body_pose_node &
ros2 run perception coordinate_transform_node &

# 4. Start control
ros2 run control pose_to_command_node &

# 5. Start UI
ros2 run ui mode_switcher_node &
ros2 run ui ui_dashboard_node

# 6. Monitor key topics
ros2 topic echo /perception/tracking_vector
ros2 topic echo /cr3/target_pose
```

### Vector Control Mode Test
```bash
# Start full pipeline (as above), then test mode switching:

# Switch to vector control mode
ros2 topic pub /mode std_msgs/String "data: 'vector_control'" --once

# Monitor robot commands
ros2 topic echo /cr3/target_pose

# Switch back to pose tracking
ros2 topic pub /mode std_msgs/String "data: 'pose_tracking'" --once
```

## Enhanced Node Parameters

### coordinate_transform_node Parameters
```bash
ros2 run perception coordinate_transform_node --ros-args -p camera_frame:=camera_link -p robot_frame:=robot_base -p scaling_factor:=0.5
```

### pose_to_command_node Parameters
```bash
ros2 run control pose_to_command_node --ros-args -p default_mode:=pose_tracking -p safety_enabled:=true
```

### ui_dashboard_node Parameters
```bash
ros2 run ui ui_dashboard_node --ros-args -p update_rate:=30.0 -p debug_mode:=true
```

## Troubleshooting

### Camera Issues
```bash
# Check camera availability
ls /dev/video*

# Test camera with OpenCV
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAILED')"
```

### OpenCV/CV Bridge Issues
```bash
# Install dependencies if missing
sudo apt update
sudo apt install python3-opencv ros-humble-cv-bridge

# Check imports
python3 -c "import cv2; from cv_bridge import CvBridge; print('OpenCV imports OK')"
```

### Topic Communication Issues
```bash
# Check if nodes can see each other
ros2 node list
ros2 topic list

# Verify topic connections
ros2 topic info /perception/hand_pose --verbose
```

### Build Issues
```bash
# Clean build if dependencies changed
rm -rf build/ install/ log/
colcon build --symlink-install

# Build with verbose output for debugging
colcon build --event-handlers console_direct+
```

## Performance Monitoring

### Real-time Topic Rates
```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /perception/hand_pose
ros2 topic hz /cr3/target_pose
```

### Node Resource Usage
```bash
# Monitor CPU/memory usage
top -p $(pgrep -f "ros2 run")

# ROS-specific monitoring
ros2 run rqt_top rqt_top
```

## Dependencies Verification

### Required System Packages
```bash
# Check if all dependencies are installed
python3 -c "
import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from geometry_msgs.msg import Pose, Vector3
print('All dependencies OK')
"
```

### Package Dependencies Status
```bash
# Check package dependencies
rosdep check --from-paths src --ignore-src

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Development Workflow

### Typical Development Cycle
1. **Modify node code**
2. **Build specific package**: `colcon build --packages-select <package_name>`
3. **Source environment**: `source install/setup.bash`
4. **Test individual node**: `ros2 run <package> <node>`
5. **Monitor topics**: `ros2 topic echo <topic_name>`
6. **Debug and iterate**

### Adding New Functionality
1. **Modify node implementation**
2. **Update package.xml if new dependencies added**
3. **Update setup.py if new console scripts added**
4. **Build and test**: `colcon build --packages-select <package>`
5. **Verify topic connectivity**: `ros2 topic list`
6. **Test integration with other nodes**

This approach provides maximum flexibility for testing, debugging, and development while maintaining full control over the system startup sequence.
