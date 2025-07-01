# CR3 Hand Tracking System - Complete Setup Guide

## Overview
This is a complete ROS 2 hand and body tracking system for the CR3 robot control. The system uses OpenCV and MediaPipe for real-time pose detection with a modular, fullscreen UI dashboard.

## System Architecture

### Core Components
1. **Camera Interface** - Real-time video capture
2. **Perception** - Hand and body pose detection using MediaPipe
3. **Control** - Robot control logic and mode management
4. **UI** - Modular fullscreen dashboard with pose visualization
5. **Diagnostics** - System monitoring and error handling
6. **CR3 Interface** - Robot hardware communication

### Key Features
- Real-time hand tracking with left/right hand selection
- Full body pose detection with 33 landmarks
- Fullscreen modular UI with live camera feed
- Interactive buttons for mode switching and hand selection
- Emergency stop functionality
- System health monitoring

## Quick Start

### 1. Build the System
```bash
cd /home/andrewlh/VSCode/ros2_ws
colcon build
source install/setup.bash
```

### 2. Validate System
```bash
./validate_system.sh
```

### 3. Run All Nodes
```bash
./run_all_nodes.sh
```

## Package Structure

### Camera Interface (`camera_interface`)
- **camera_node.py** - Captures video from camera and publishes to `/camera/image_raw`
- Optimized for 30 FPS with minimal latency
- Auto-detects camera or falls back to test patterns

### Perception (`perception`)
- **hand_pose_node.py** - MediaPipe hand tracking
  - Publishes 21 hand landmarks to `/perception/hand_pose`
  - Supports left/right hand selection via `/ui/hand_selection`
  - Publishes confidence to `/perception/hand_confidence`
- **body_pose_node.py** - MediaPipe body tracking
  - Publishes 33 body landmarks to `/perception/body_pose`
  - Optimized for real-time performance
- **coordinate_transform_node.py** - Minimal coordinate transformation stub
- **pose_filter_node.py** - Pose filtering stub (outline only)

### Control (`control`)
- **pose_to_command_node.py** - Converts poses to robot commands (minimal)
- **motion_planner_node.py** - Motion planning stub
- **trajectory_executor_node.py** - Trajectory execution stub
- **teleop_node.py** - Manual control stub

### UI (`ui`) - Modular Design
- **ui_dashboard_node_modular.py** - Main dashboard coordinator
- **ui_window_manager.py** - Fullscreen window management and scaling
- **ui_button_controller.py** - Button interactions and event handling
- **ui_visualization.py** - Pose overlay visualization
- **mode_switcher_node.py** - Mode switching logic

### Diagnostics (`diagnostics`)
- Various monitoring nodes (stubs for system health)

### CR3 Interface (`cr3_interface`)
- Robot hardware communication nodes (stubs)

## Topics and Communication

### Camera Topics
- `/camera/image_raw` (sensor_msgs/Image) - Live camera feed

### Perception Topics
- `/perception/hand_pose` (geometry_msgs/PoseArray) - 21 hand landmarks
- `/perception/hand_confidence` (std_msgs/Float32) - Hand detection confidence
- `/perception/body_pose` (geometry_msgs/PoseArray) - 33 body landmarks

### Control Topics
- `/mode` (std_msgs/String) - Current control mode
- `/ui/hand_selection` (std_msgs/String) - Selected hand ("Left" or "Right")
- `/ui/command` (std_msgs/String) - UI commands
- `/emergency_stop` (std_msgs/Bool) - Emergency stop state

## UI Controls

### Mouse Controls
- **PAUSE/RESUME** - Pause/resume system
- **EMERGENCY STOP** - Activate emergency stop
- **CYCLE MODE** - Switch between control modes
- **LEFT/RIGHT** - Select which hand to track
- **STOP ALL NODES** - Shutdown entire system

### Keyboard Shortcuts
- **Q or ESC** - Quit application
- **P** - Pause/resume
- **E** - Emergency stop
- **M** - Cycle mode
- **L** - Select left hand
- **R** - Select right hand
- **S** - Stop all nodes

## Configuration

### Hand Tracking
- Default: Right hand tracking
- Switch via UI buttons or keyboard shortcuts
- MediaPipe confidence thresholds optimized for real-time performance

### Display
- Fullscreen UI with automatic scaling
- Camera feed on left, control panel on right
- Real-time pose overlays with landmark numbering

### Performance
- Camera: 30 FPS capture
- Hand pose: Real-time processing (no frame skipping)
- Body pose: Real-time processing (no frame skipping)
- UI: 30 FPS updates

## File Structure
```
/home/andrewlh/VSCode/ros2_ws/
├── src/
│   ├── camera_interface/
│   ├── perception/
│   ├── control/
│   ├── ui/
│   ├── diagnostics/
│   ├── cr3_interface/
│   └── sim_interface/
├── documentation/
├── run_all_nodes.sh
└── validate_system.sh
```

## Essential Files

### Executables
- `run_all_nodes.sh` - Starts all system nodes in correct order
- `validate_system.sh` - Validates system configuration and syntax

### Key Nodes
- `camera_interface/camera_node.py` - Camera capture
- `perception/hand_pose_node.py` - Hand tracking
- `perception/body_pose_node.py` - Body tracking
- `ui/ui_dashboard_node_modular.py` - Main UI
- `ui/mode_switcher_node.py` - Mode management

## Dependencies
- ROS 2 Jazzy
- OpenCV (cv2)
- MediaPipe
- NumPy
- Python 3.12+

## Performance Notes
- System optimized for real-time responsiveness
- No frame skipping on pose detection for minimal lag
- Fullscreen UI with proper scaling for all screen sizes
- Direct camera access with minimal buffering

## Troubleshooting

### No Camera Feed
- Check camera permissions and connections
- Verify camera is not in use by another application
- Check console output for camera initialization errors

### Poor Performance
- Ensure adequate CPU/GPU resources
- Check for other resource-intensive applications
- Monitor system logs for errors

### Hand Detection Issues
- Ensure good lighting conditions
- Check hand is clearly visible in camera frame
- Verify correct hand selection (left/right)

## Development Notes
- All launch files removed - use direct node execution only
- Modular UI design for easy maintenance and extension
- Minimal stub implementations for future expansion
- Comprehensive error handling and logging

## Future Enhancements
- Robot control integration
- Advanced pose filtering
- Multi-user support
- Recording and playback capabilities
