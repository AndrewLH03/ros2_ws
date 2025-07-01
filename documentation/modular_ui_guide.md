# Modular UI Dashboard for CR3 Hand Tracking System

## Overview
The UI dashboard has been completely refactored into a modular architecture with the following components:

### Architecture

#### 1. **ui_window_manager.py**
- Manages the main OpenCV window
- Handles fullscreen mode and responsive scaling
- Automatically detects screen resolution
- Scales UI elements proportionally

#### 2. **ui_button_controller.py** 
- Manages all button logic and interactions
- Handles mouse clicks and keyboard shortcuts
- Contains all button action implementations
- Supports responsive button sizing

#### 3. **ui_visualization.py**
- Handles pose visualization and overlays
- Draws hand and body pose landmarks
- Manages camera feed processing
- Supports scaled visualization elements

#### 4. **ui_dashboard_node_modular.py**
- Main coordination node
- Integrates all modular components
- Manages ROS topic subscriptions/publications
- Coordinates the display loop

## Features

### Fullscreen Mode
- **Automatic fullscreen startup**: The UI opens in fullscreen mode by default
- **Responsive scaling**: All elements scale proportionally to screen size
- **Toggle fullscreen**: Press `F` to toggle between fullscreen and windowed mode

### Control Options

#### Mouse Controls
- **Pause/Resume**: Click to pause/resume the UI
- **Emergency Stop**: Red button to trigger emergency stop
- **Cycle Mode**: Blue button to cycle through control modes
- **Hand Selection**: Left/Right buttons to select which hand to track
- **Stop All Nodes**: Red button to shutdown all ROS nodes

#### Keyboard Shortcuts
- `Q` or `ESC`: Quit the application
- `P`: Pause/resume the UI
- `E`: Toggle emergency stop
- `M`: Cycle through modes
- `L`: Select left hand tracking
- `R`: Select right hand tracking
- `S`: Stop all nodes
- `F`: Toggle fullscreen mode

### Status Display
- **Current mode**: Shows active control mode
- **Hand confidence**: Real-time confidence score
- **Selected hand**: Which hand is being tracked
- **Landmark counts**: Number of detected hand/body landmarks
- **System status**: Emergency stop status
- **FPS counter**: Real-time performance indicator

## Usage

### Starting the Modular UI
```bash
# Source the workspace
source install/setup.bash

# Run the modular UI dashboard
ros2 run ui ui_dashboard_node
```

### Integration with Hand Tracking
The modular UI works with the existing hand tracking system:

```bash
# Terminal 1: Camera
ros2 run camera_interface camera_node

# Terminal 2: Hand pose detection  
ros2 run perception hand_pose_node

# Terminal 3: Body pose detection
ros2 run perception body_pose_node

# Terminal 4: Modular UI (fullscreen)
ros2 run ui ui_dashboard_node
```

### Customization

#### Scaling Factors
The UI automatically detects screen resolution and scales appropriately. You can modify scaling behavior in `ui_window_manager.py`:

```python
# Adjust panel width ratio (default 25% of screen)
self.panel_width_ratio = 0.25  

# Modify scaling factors in get_scaling_factors()
```

#### Button Layout
Modify button positions and sizes in `ui_button_controller.py`:

```python
# Adjust base button dimensions
self.base_button_width = 180
self.base_button_height = 40
self.base_button_spacing = 50
```

#### Visualization
Customize pose visualization in `ui_visualization.py`:

```python
# Adjust landmark sizes and colors
# Modify connection thickness
# Change text overlay positions
```

## Benefits of Modular Architecture

1. **Maintainability**: Each component has a specific responsibility
2. **Testability**: Individual modules can be tested separately  
3. **Scalability**: Easy to add new features to specific modules
4. **Reusability**: Modules can be reused in other applications
5. **Performance**: Better code organization and optimization

## Migration from Old UI

The original `ui_dashboard_node.py` has been backed up as `ui_dashboard_node_old.py`. The new modular system provides the same functionality with:

- Better organization
- Fullscreen support
- Responsive scaling
- Improved performance
- Easier maintenance

## Troubleshooting

### Common Issues

1. **Screen size detection fails**: 
   - Check if OpenCV can access display
   - Ensure proper graphics drivers

2. **Scaling looks wrong**:
   - Verify screen resolution detection
   - Adjust scaling factors manually if needed

3. **Performance issues**:
   - Check FPS counter in top-right
   - Reduce visualization complexity if needed

### Debug Mode
Enable debug information by setting `show_debug=True` in the UI state to see landmark numbers and additional information.
