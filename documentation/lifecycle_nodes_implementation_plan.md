# ROS2 Lifecycle Nodes Implementation Plan

## Table of Contents
1. [Overview](#overview)
2. [Benefits of Lifecycle Nodes](#benefits-of-lifecycle-nodes)
3. [Lifecycle States and Transitions](#lifecycle-states-and-transitions)
4. [Implementation Strategy](#implementation-strategy)
5. [Phase-by-Phase Conversion Plan](#phase-by-phase-conversion-plan)
6. [Code Conversion Guidelines](#code-conversion-guidelines)
7. [Node-Specific Conversion Details](#node-specific-conversion-details)
8. [Testing and Validation](#testing-and-validation)
9. [Launch File Updates](#launch-file-updates)
10. [Troubleshooting Guide](#troubleshooting-guide)

## Overview

This document outlines the comprehensive plan for converting all nodes in the CR3 robotic control system from standard ROS2 nodes to lifecycle nodes. This conversion will provide better control over node states, enable clean mode switching, improve error handling, and allow for more efficient resource management.

### Current System Architecture
```
Current Nodes (Standard ROS2 Nodes):
├── control_system/
│   ├── unified_control_node
│   ├── manual_control_node
│   ├── perception_control_node
│   ├── servo_interface_node
│   └── mode_manager_node
├── camera_interface/
│   ├── camera_node
│   └── camera_info_node
├── perception/
│   ├── hand_pose_node
│   ├── body_pose_node
│   ├── pose_filter_node
│   └── coordinate_transform_node
├── servo_control/
│   └── finger_servo_controller_node
├── cr3_interface/
│   ├── cr3_controller_node
│   ├── hand_controller_node
│   ├── ip_to_ros_bridge_node
│   ├── joint_state_publisher_node
│   └── tf_broadcaster_node
├── diagnostics/
│   ├── emergency_stop_node
│   ├── error_handler_node
│   ├── health_monitor_node
│   ├── logger_node
│   └── watchdog_node
├── sim_interface/
│   ├── sim_world_interface_node
│   └── simulator_node
└── ui/
    ├── mode_switcher_node
    └── ui_dashboard_node
```

### Target System Architecture
```
Target Nodes (Lifecycle Nodes):
├── lifecycle_manager (Central Coordinator)
├── control_system/ (Lifecycle Nodes)
├── camera_interface/ (Lifecycle Nodes)
├── perception/ (Lifecycle Nodes)
├── servo_control/ (Lifecycle Nodes)
├── cr3_interface/ (Lifecycle Nodes)
├── diagnostics/ (Lifecycle Nodes)
├── sim_interface/ (Lifecycle Nodes)
└── ui/ (Lifecycle Nodes)
```

## Benefits of Lifecycle Nodes

### 1. **Controlled Startup/Shutdown Sequences**
- Nodes start in proper dependency order
- Clean shutdown prevents resource leaks
- Hardware interfaces only initialize when needed

### 2. **Mode Switching Capabilities**
- **Manual Mode**: Only manual control and servo nodes active
- **Perception Mode**: Camera, hand pose, and perception nodes active
- **Simulation Mode**: Only simulation nodes active
- **Diagnostic Mode**: All monitoring nodes active

### 3. **Resource Management**
- Camera only captures when in perception mode
- Servo controllers only active when needed
- Network connections managed efficiently

### 4. **Error Recovery**
- Individual nodes can be restarted without system shutdown
- Failed nodes don't affect other components
- Graceful degradation of functionality

### 5. **Debugging and Testing**
- Individual components can be isolated
- State inspection capabilities
- Easier unit testing

## Lifecycle States and Transitions

### Standard Lifecycle States
```
┌─────────────────┐    configure()    ┌─────────────────┐
│   Unconfigured  │ ─────────────────→│     Inactive    │
│                 │←───────────────── │                 │
└─────────────────┘    cleanup()      └─────────────────┘
                                              │ ▲
                                   activate() │ │ deactivate()
                                              ▼ │
                                      ┌─────────────────┐
                                      │     Active      │
                                      │                 │
                                      └─────────────────┘
                                              │
                                   shutdown() │
                                              ▼
                                      ┌─────────────────┐
                                      │   Finalized     │
                                      │                 │
                                      └─────────────────┘
```

### State Descriptions
- **Unconfigured**: Node exists but resources not allocated
- **Inactive**: Node configured but not processing data
- **Active**: Node fully operational and processing
- **Finalized**: Node shut down and cleaned up

### Transition Callbacks
- `on_configure()`: Initialize resources, create publishers/subscribers
- `on_activate()`: Start data processing, activate publishers
- `on_deactivate()`: Stop processing, deactivate publishers
- `on_cleanup()`: Clean up resources, destroy publishers/subscribers
- `on_shutdown()`: Final cleanup before node destruction

## Implementation Strategy

### Phase 1: Infrastructure Setup (Week 1)
1. **Update Package Dependencies**
2. **Create Lifecycle Manager**
3. **Update Launch Files**
4. **Create Testing Framework**

### Phase 2: Core Nodes Conversion (Week 2)
1. **Servo Interface Node** (Hardware critical)
2. **Camera Interface Nodes** (Resource intensive)
3. **Mode Manager Node** (Central coordination)

### Phase 3: Control Nodes Conversion (Week 3)
1. **Manual Control Node**
2. **Perception Control Node**
3. **Unified Control Node**

### Phase 4: Perception Nodes Conversion (Week 4)
1. **Hand Pose Node**
2. **Body Pose Node**
3. **Pose Filter Node**
4. **Coordinate Transform Node**

### Phase 5: Supporting Nodes Conversion (Week 5)
1. **CR3 Interface Nodes**
2. **Diagnostic Nodes**
3. **UI Nodes**
4. **Simulation Nodes**

### Phase 6: Integration and Testing (Week 6)
1. **System Integration Testing**
2. **Mode Switching Validation**
3. **Error Recovery Testing**
4. **Performance Optimization**

## Code Conversion Guidelines

### 1. **Basic Node Structure Conversion**

#### Before (Standard Node):
```python
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(String, 'input', self.callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
```

#### After (Lifecycle Node):
```python
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import LifecyclePublisher

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_node')
        # Don't create publishers/subscribers here
        self.publisher = None
        self.subscription = None
        self.timer = None
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        self.publisher = self.create_lifecycle_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(String, 'input', self.callback, 10)
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')
        self.publisher.on_activate()
        self.timer = self.create_timer(1.0, self.timer_callback)
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        self.publisher.on_deactivate()
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        if self.publisher:
            self.destroy_lifecycle_publisher(self.publisher)
        if self.subscription:
            self.destroy_subscription(self.subscription)
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS
```

### 2. **Publisher Conversion**

#### Standard Publisher → Lifecycle Publisher:
```python
# Configure phase
self.pose_pub = self.create_lifecycle_publisher(PoseArray, '/perception/hand_pose', 10)

# Activate phase
self.pose_pub.on_activate()

# Publishing (only when active)
if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
    self.pose_pub.publish(msg)

# Deactivate phase
self.pose_pub.on_deactivate()
```

### 3. **Hardware Resource Management**

```python
def on_configure(self, state):
    """Initialize hardware connections."""
    try:
        # Initialize camera/servo connections
        self.camera = cv2.VideoCapture(0)
        return TransitionCallbackReturn.SUCCESS
    except Exception as e:
        self.get_logger().error(f'Hardware init failed: {e}')
        return TransitionCallbackReturn.FAILURE

def on_activate(self, state):
    """Start hardware operations."""
    # Start camera capture, servo control, etc.
    return TransitionCallbackReturn.SUCCESS

def on_deactivate(self, state):
    """Stop hardware operations."""
    # Stop camera, servos, but keep connections
    return TransitionCallbackReturn.SUCCESS

def on_cleanup(self, state):
    """Release hardware resources."""
    if self.camera:
        self.camera.release()
    return TransitionCallbackReturn.SUCCESS
```

### 4. **Error Handling Pattern**

```python
def on_configure(self, state):
    try:
        # Configuration logic
        self.get_logger().info('Configuration successful')
        return TransitionCallbackReturn.SUCCESS
    except Exception as e:
        self.get_logger().error(f'Configuration failed: {e}')
        return TransitionCallbackReturn.FAILURE

def callback_with_state_check(self, msg):
    """Only process callbacks when active."""
    if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
        return
    
    # Process message only when active
    # ...existing logic...
```

## Node-Specific Conversion Details

### 1. **Camera Interface Nodes**

#### camera_node.py conversion:
```python
class CameraLifecycleNode(LifecycleNode):
    def on_configure(self, state):
        # Initialize camera connection
        self.camera = cv2.VideoCapture(0)
        self.image_pub = self.create_lifecycle_publisher(Image, '/camera/image', 10)
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        # Start image capture timer
        self.image_pub.on_activate()
        self.timer = self.create_timer(0.033, self.capture_image)  # 30 FPS
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        # Stop capture but keep camera connection
        self.image_pub.on_deactivate()
        if self.timer:
            self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS
```

### 2. **Hand Pose Node**

#### hand_pose_node.py conversion:
```python
class HandPoseLifecycleNode(LifecycleNode):
    def on_configure(self, state):
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(...)
        
        # Create publishers
        self.pose_pub = self.create_lifecycle_publisher(PoseArray, '/perception/hand_pose', 10)
        self.confidence_pub = self.create_lifecycle_publisher(Float32, '/perception/hand_confidence', 10)
        
        # Create subscription
        self.image_sub = self.create_subscription(Image, '/camera/image', self.process_image, 10)
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        # Activate publishers
        self.pose_pub.on_activate()
        self.confidence_pub.on_activate()
        return TransitionCallbackReturn.SUCCESS
    
    def process_image(self, msg):
        # Only process when active
        if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return
        # ...existing processing logic...
```

### 3. **Servo Control Nodes**

#### servo_interface_node.py conversion:
```python
class ServoInterfaceLifecycleNode(LifecycleNode):
    def on_configure(self, state):
        # Initialize servo connections
        self.servo_connections = {}
        for servo_id in self.servo_ids:
            self.servo_connections[servo_id] = ServoConnection(servo_id)
        
        # Create command subscription
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, '/servo/commands', self.servo_callback, 10)
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        # Enable servo power/control
        for servo in self.servo_connections.values():
            servo.enable()
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        # Disable servo movement but keep connections
        for servo in self.servo_connections.values():
            servo.disable()
        return TransitionCallbackReturn.SUCCESS
```

### 4. **Mode Manager Node**

#### mode_manager_node.py conversion:
```python
class ModeManagerLifecycleNode(LifecycleNode):
    def on_configure(self, state):
        # Create mode management services
        self.mode_pub = self.create_lifecycle_publisher(String, '/current_mode', 10)
        self.mode_sub = self.create_subscription(String, '/mode_request', self.handle_mode_change, 10)
        
        # Initialize lifecycle management clients
        self.node_clients = {}
        for node_name in self.managed_nodes:
            self.node_clients[node_name] = self.create_client(
                ChangeState, f'/{node_name}/change_state')
        return TransitionCallbackReturn.SUCCESS
    
    def handle_mode_change(self, msg):
        """Handle mode switching with lifecycle management."""
        if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return
        
        new_mode = msg.data
        self.get_logger().info(f'Switching to {new_mode} mode')
        
        # Deactivate current mode nodes
        self.deactivate_mode_nodes(self.current_mode)
        
        # Activate new mode nodes
        self.activate_mode_nodes(new_mode)
        
        self.current_mode = new_mode
```

## Testing and Validation

### 1. **Unit Testing for Lifecycle Nodes**

Create test file: `test/test_lifecycle_nodes.py`
```python
import unittest
import rclpy
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn
from your_package.hand_pose_lifecycle_node import HandPoseLifecycleNode

class TestHandPoseLifecycle(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = HandPoseLifecycleNode()
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_configure_transition(self):
        """Test configuration transition."""
        result = self.node.on_configure(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        # Verify resources are initialized
        self.assertIsNotNone(self.node.pose_pub)
    
    def test_activate_transition(self):
        """Test activation transition."""
        # Configure first
        self.node.on_configure(None)
        result = self.node.on_activate(None)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        # Verify publishers are active
        self.assertTrue(self.node.pose_pub.is_activated())
    
    def test_full_lifecycle(self):
        """Test complete lifecycle sequence."""
        # Configure → Activate → Deactivate → Cleanup
        self.assertEqual(self.node.on_configure(None), TransitionCallbackReturn.SUCCESS)
        self.assertEqual(self.node.on_activate(None), TransitionCallbackReturn.SUCCESS)
        self.assertEqual(self.node.on_deactivate(None), TransitionCallbackReturn.SUCCESS)
        self.assertEqual(self.node.on_cleanup(None), TransitionCallbackReturn.SUCCESS)
```

### 2. **Integration Testing**

Create test script: `test/test_mode_switching.py`
```python
#!/usr/bin/env python3
"""Integration test for mode switching with lifecycle nodes."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState
import time

class LifecycleTester(Node):
    def __init__(self):
        super().__init__('lifecycle_tester')
        self.mode_pub = self.create_publisher(String, '/mode_request', 10)
        
    def test_mode_switching(self):
        """Test switching between manual and perception modes."""
        # Switch to perception mode
        mode_msg = String()
        mode_msg.data = 'perception'
        self.mode_pub.publish(mode_msg)
        
        time.sleep(2.0)  # Allow mode switch
        
        # Verify perception nodes are active
        perception_nodes = ['camera_node', 'hand_pose_node']
        for node_name in perception_nodes:
            state = self.get_node_state(node_name)
            assert state == 'active', f'{node_name} should be active in perception mode'
        
        # Switch to manual mode
        mode_msg.data = 'manual'
        self.mode_pub.publish(mode_msg)
        
        time.sleep(2.0)
        
        # Verify perception nodes are inactive
        for node_name in perception_nodes:
            state = self.get_node_state(node_name)
            assert state == 'inactive', f'{node_name} should be inactive in manual mode'
```

### 3. **Performance Testing**

Monitor resource usage during mode switches:
```bash
# Monitor CPU/Memory during mode switches
top -p $(pgrep -f "lifecycle_manager|camera_node|hand_pose")

# Monitor ROS2 topic throughput
ros2 topic hz /camera/image
ros2 topic hz /perception/hand_pose
```

## Launch File Updates

### 1. **Updated Launch File Structure**

Create `launch/lifecycle_nodes.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Lifecycle Manager (standard node)
    lifecycle_manager = Node(
        package='control_system',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen'
    )
    
    # Camera Node (lifecycle)
    camera_node = LifecycleNode(
        package='camera_interface',
        executable='camera_lifecycle_node',
        name='camera_node',
        output='screen'
    )
    
    # Hand Pose Node (lifecycle)
    hand_pose_node = LifecycleNode(
        package='perception',
        executable='hand_pose_lifecycle_node',
        name='hand_pose_node',
        output='screen'
    )
    
    # Auto-configure nodes on startup
    configure_camera = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=camera_node,
            start_state='configuring',
            goal_state='configured',
            entities=[
                ChangeState(
                    lifecycle_node_matcher=lambda node: node.name == 'camera_node',
                    transition_id=Transition.TRANSITION_CONFIGURE
                )
            ]
        )
    )
    
    return LaunchDescription([
        lifecycle_manager,
        camera_node,
        hand_pose_node,
        configure_camera,
    ])
```

### 2. **Mode-Specific Launch Files**

Create separate launch files for different modes:

#### `launch/manual_mode.launch.py`:
```python
def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='control_system',
            executable='manual_control_lifecycle_node',
            name='manual_control_node'
        ),
        LifecycleNode(
            package='servo_control',
            executable='servo_interface_lifecycle_node',
            name='servo_interface_node'
        ),
        # Auto-activate manual mode nodes
        # ...
    ])
```

#### `launch/perception_mode.launch.py`:
```python
def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='camera_interface',
            executable='camera_lifecycle_node',
            name='camera_node'
        ),
        LifecycleNode(
            package='perception',
            executable='hand_pose_lifecycle_node',
            name='hand_pose_node'
        ),
        # Auto-activate perception mode nodes
        # ...
    ])
```

## Troubleshooting Guide

### Common Issues and Solutions

#### 1. **Node Fails to Configure**
```bash
# Check node logs
ros2 lifecycle get /node_name
ros2 log view /node_name

# Common causes:
# - Missing dependencies
# - Hardware not available
# - Permission issues
```

#### 2. **Publishers Not Activated**
```python
# Verify publisher activation in node code
def on_activate(self, state):
    self.publisher.on_activate()  # This line is crucial
    return TransitionCallbackReturn.SUCCESS
```

#### 3. **Mode Switching Fails**
```bash
# Check lifecycle manager status
ros2 lifecycle list
ros2 lifecycle get /lifecycle_manager

# Manual node state changes for debugging
ros2 lifecycle set /node_name configure
ros2 lifecycle set /node_name activate
```

#### 4. **Performance Issues**
- **Camera node consuming CPU when inactive**: Ensure timer is destroyed in `on_deactivate()`
- **Memory leaks**: Verify all resources are cleaned up in `on_cleanup()`
- **Slow mode switching**: Check for blocking operations in transition callbacks

### Debugging Commands

```bash
# List all lifecycle nodes
ros2 lifecycle list

# Get node state
ros2 lifecycle get /node_name

# Manual state transitions
ros2 lifecycle set /node_name configure
ros2 lifecycle set /node_name activate
ros2 lifecycle set /node_name deactivate
ros2 lifecycle set /node_name cleanup

# Monitor state changes
ros2 topic echo /node_name/transition_event

# Check service availability
ros2 service list | grep lifecycle
```

## Implementation Checklist

### Phase 1: Infrastructure
- [ ] Update package.xml dependencies
- [ ] Create lifecycle_manager node
- [ ] Update setup.py entry points
- [ ] Create test framework

### Phase 2: Core Nodes
- [ ] Convert servo_interface_node
- [ ] Convert camera_node
- [ ] Convert camera_info_node
- [ ] Convert mode_manager_node

### Phase 3: Control Nodes
- [ ] Convert manual_control_node
- [ ] Convert perception_control_node
- [ ] Convert unified_control_node

### Phase 4: Perception Nodes
- [ ] Convert hand_pose_node
- [ ] Convert body_pose_node
- [ ] Convert pose_filter_node
- [ ] Convert coordinate_transform_node

### Phase 5: Supporting Nodes
- [ ] Convert CR3 interface nodes
- [ ] Convert diagnostic nodes
- [ ] Convert UI nodes
- [ ] Convert simulation nodes

### Phase 6: Integration
- [ ] Update all launch files
- [ ] Test mode switching
- [ ] Test error recovery
- [ ] Performance validation
- [ ] Documentation update

## Dependencies to Add

Update `package.xml` for all packages:
```xml
<depend>lifecycle_msgs</depend>
<depend>rclpy_lifecycle</depend>
```

Update `setup.py` for Python packages:
```python
install_requires=[
    'setuptools',
    'rclpy',
    'lifecycle_msgs',
]
```

## Timeline and Milestones

### Week 1: Infrastructure Setup
- **Day 1-2**: Update dependencies, create lifecycle manager
- **Day 3-4**: Create testing framework
- **Day 5**: Update launch files structure

### Week 2: Core Nodes (25% completion)
- **Day 1-2**: Convert servo interface and camera nodes
- **Day 3-4**: Convert mode manager
- **Day 5**: Integration testing

### Week 3: Control Nodes (50% completion)
- **Day 1-3**: Convert all control nodes
- **Day 4-5**: Mode switching implementation

### Week 4: Perception Nodes (75% completion)
- **Day 1-3**: Convert perception nodes
- **Day 4-5**: Perception mode testing

### Week 5: Supporting Nodes (90% completion)
- **Day 1-3**: Convert remaining nodes
- **Day 4-5**: System integration

### Week 6: Final Testing (100% completion)
- **Day 1-3**: End-to-end testing
- **Day 4-5**: Performance optimization and documentation

## Success Criteria

1. **Functional Requirements**
   - [ ] All nodes successfully convert to lifecycle nodes
   - [ ] Mode switching works reliably
   - [ ] Error recovery functions properly
   - [ ] Clean startup/shutdown sequences

2. **Performance Requirements**
   - [ ] No performance degradation in active modes
   - [ ] Resource usage minimized in inactive modes
   - [ ] Mode switching completes within 2 seconds

3. **Reliability Requirements**
   - [ ] System recovers from individual node failures
   - [ ] No memory leaks during mode switching
   - [ ] Consistent behavior across multiple cycles

This comprehensive plan ensures a systematic and thorough conversion to lifecycle nodes while maintaining system functionality and improving overall robustness.
