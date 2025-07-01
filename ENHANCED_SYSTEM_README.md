# Enhanced CR3 Hand Tracking System - Setup and Validation Guide

## Overview

This system integrates OpenCV/MediaPipe-based hand tracking dashboard functionality into the ROS 2 workspace, providing real-time pose tracking, coordinate transformation, and interactive control capabilities.

## System Architecture

```
Camera Feed → hand_pose_node → coordinate_transform_node → pose_to_command_node → Robot Commands
                    ↓               ↓                           ↓
            body_pose_node → UI Dashboard ←------ mode_switcher_node
                    ↓               ↓                           ↓
                Visualization  Real-time Status         Interactive Control
```

## New Features and Enhanced Nodes

### Enhanced Nodes:
- **coordinate_transform_node**: Full implementation with vector calculation and coordinate transformation
- **pose_to_command_node**: Enhanced with vector-based control modes and dynamic scaling
- **ui_dashboard_node**: Complete OpenCV dashboard with real-time visualization
- **mode_switcher_node**: Enhanced with new control modes and UI integration

### New UI Components:
- **ui_components.py**: Modular OpenCV UI library for dashboard functionality

### New Topics:
```
/perception/hand_confidence     - Float32    (hand detection confidence)
/perception/shoulder_pose       - Pose       (shoulder position in robot frame)
/perception/wrist_pose         - Pose       (wrist position in robot frame)
/perception/tracking_vector    - Vector3    (shoulder-to-wrist vector)
/ui/control_commands          - String     (UI control commands)
/ui/visualization_frame       - Image      (processed camera feed)
```

### Enhanced Control Modes:
- `manual`: Manual control with safety heartbeat
- `pose_tracking`: Direct pose mapping with scaling
- `vector_control`: Vector-magnitude-based control (NEW)
- `autonomous`: Future expansion capability

## Dependencies

### Python Dependencies (automatically handled by package.xml):
- `python3-opencv`: OpenCV for computer vision
- `python3-numpy`: Numerical computations
- `cv-bridge`: ROS-OpenCV interface

### ROS 2 Message Dependencies:
- `geometry_msgs`: Pose, Vector3, PoseArray
- `sensor_msgs`: Image, JointState
- `std_msgs`: String, Bool, Float32

## Installation and Setup

### 1. Build the Enhanced System
```bash
cd /home/andrewlh/VSCode/ros2_ws
colcon build --packages-select perception control ui camera_interface
source install/setup.bash
```

### 2. Validate System Configuration
```bash
# Run comprehensive system validation
python3 system_validation.py
```

This script will verify:
- ✅ All enhanced Python modules can be imported
- ✅ All node executables are properly registered
- ✅ All message types are available
- ✅ All dependencies are installed

### 3. Launch Individual Components

#### Perception System:
```bash
ros2 launch perception.launch.py
```

#### Control System:
```bash
ros2 launch control.launch.py
```

#### UI Dashboard:
```bash
ros2 launch ui_dashboard.launch.py
```

#### Complete System:
```bash
ros2 launch bringup.launch.py
```

### 4. Launch with Custom Parameters

#### With real camera:
```bash
ros2 launch bringup.launch.py use_camera:=true camera_device:=0
```

#### With specific control mode:
```bash
ros2 launch bringup.launch.py default_mode:=vector_control
```

#### With dashboard disabled:
```bash
ros2 launch bringup.launch.py enable_dashboard:=false
```

## Testing and Validation

### 1. Check Node Status
```bash
ros2 node list
# Expected nodes:
# /camera_node
# /hand_pose_node  
# /body_pose_node
# /coordinate_transform_node
# /pose_to_command_node
# /ui_dashboard_node
# /mode_switcher_node
```

### 2. Check Topic Flow
```bash
ros2 topic list
# Expected new topics:
# /perception/hand_confidence
# /perception/shoulder_pose
# /perception/wrist_pose
# /perception/tracking_vector
# /ui/control_commands
# /ui/visualization_frame
```

### 3. Test Topic Communication
```bash
# Monitor hand confidence
ros2 topic echo /perception/hand_confidence

# Monitor tracking vector
ros2 topic echo /perception/tracking_vector

# Test mode switching
ros2 topic pub /ui/control_commands std_msgs/String "data: 'set_mode:vector_control'"
```

### 4. Interactive Dashboard Testing

When the dashboard is running:
- **Mouse Controls**: Click buttons in the UI panel
- **Keyboard Controls**:
  - `q`: Quit application
  - `p`: Pause/Resume tracking
  - `m`: Cycle control modes
  - `e`: Emergency stop

## Package Structure Validation

### Verified Package Configurations:

#### perception/package.xml ✅
- Added `std_msgs` dependency
- Added `python3-numpy` exec dependency

#### ui/package.xml ✅  
- Added OpenCV dependencies: `python3-opencv`, `cv-bridge`
- Added `python3-numpy` exec dependency

#### Launch Files ✅
- `perception.launch.py`: Complete perception pipeline
- `control.launch.py`: Enhanced control system
- `ui_dashboard.launch.py`: Dashboard interface
- `bringup.launch.py`: Complete system launch

#### Console Scripts ✅
All enhanced nodes properly registered in setup.py files:
- `coordinate_transform_node = perception.coordinate_transform_node:main`
- `pose_to_command_node = control.pose_to_command_node:main`
- `ui_dashboard_node = ui.ui_dashboard_node:main`
- `mode_switcher_node = ui.mode_switcher_node:main`

## Troubleshooting

### Common Issues:

1. **Import Errors**: 
   ```bash
   # Rebuild packages
   colcon build --packages-select perception control ui
   source install/setup.bash
   ```

2. **OpenCV Not Found**:
   ```bash
   sudo apt update
   sudo apt install python3-opencv
   ```

3. **Topics Not Appearing**:
   - Check if all nodes are running: `ros2 node list`
   - Verify topic remapping in launch files
   - Check node logs: `ros2 log view`

4. **Dashboard Not Opening**:
   - Ensure X11 forwarding if using SSH
   - Check camera permissions: `ls -l /dev/video*`
   - Verify OpenCV installation: `python3 -c "import cv2; print(cv2.__version__)"`

### Debug Mode:
```bash
# Launch with debug logging
ros2 launch bringup.launch.py --ros-args --log-level debug
```

## System Status Check

Run the validation script anytime to check system health:
```bash
python3 system_validation.py
```

Expected output:
```
🎉 ALL TESTS PASSED - System is ready for deployment!
```

## Next Steps

1. **Hardware Integration**: Connect to actual CR3 robot interface
2. **Performance Tuning**: Optimize for target hardware specifications  
3. **Custom Configuration**: Adjust parameters in launch files for specific use cases
4. **Production Deployment**: Use launch files for automated system startup

## Support

For issues or questions:
1. Check the validation script output for specific errors
2. Review node logs: `ros2 log view`
3. Verify all dependencies are installed
4. Ensure proper ROS 2 workspace sourcing
