# Finger Position Extraction and Servo Control Architecture

## Overview

This document outlines the architecture for extracting finger positions from hand pose data and controlling Dynamixel XL330-288 servos for robotic hand mimicking. It details how the hand pose node can be enhanced versus creating separate processing nodes.

## Current System Analysis

### Hand Pose Node Current Capabilities
- **MediaPipe Integration**: Already processes hand landmarks (21 points per hand)
- **Hand Selection**: Supports left/right hand preference 
- **Real-time Processing**: Optimized for minimal latency
- **Landmark Publishing**: Publishes all 21 landmarks as PoseArray

### Pose To Command Node Current State
- **Minimal Implementation**: Currently just a stub for mode handling
- **Future Framework**: Designed as placeholder for robot command generation
- **Mode Management**: Handles control mode switching

## Architecture Decision: Enhanced vs Complementary Approach

### Option 1: Enhanced Hand Pose Node (Recommended)
**Advantages:**
- Single point of hand data processing
- Eliminates data pipeline latency
- Direct access to MediaPipe calculations
- Simplified architecture

**Implementation:**
```python
class HandPoseNode(Node):
    """
    Enhanced hand pose detection with finger position extraction.
    - Processes MediaPipe hand landmarks
    - Calculates finger joint angles and positions
    - Publishes both raw landmarks and processed finger data
    """
    
    def __init__(self):
        # ...existing code...
        
        # Additional publishers for finger data
        self.finger_positions_pub = self.create_publisher(
            Float32MultiArray, '/perception/finger_positions', 10)
        self.finger_angles_pub = self.create_publisher(
            Float32MultiArray, '/perception/finger_angles', 10)
        self.finger_states_pub = self.create_publisher(
            FingerStates, '/perception/finger_states', 10)
    
    def extract_and_publish_hand_poses(self, results, image_shape):
        # ...existing landmark processing...
        
        # NEW: Extract finger positions
        if preferred_hand_landmarks:
            finger_data = self.extract_finger_positions(preferred_hand_landmarks)
            self.publish_finger_data(finger_data)
    
    def extract_finger_positions(self, hand_landmarks):
        """Extract finger-specific data from hand landmarks."""
        finger_data = {
            'thumb': self.process_finger(hand_landmarks, [1, 2, 3, 4]),
            'index': self.process_finger(hand_landmarks, [5, 6, 7, 8]),
            'middle': self.process_finger(hand_landmarks, [9, 10, 11, 12]),
            'ring': self.process_finger(hand_landmarks, [13, 14, 15, 16]),
            'pinky': self.process_finger(hand_landmarks, [17, 18, 19, 20])
        }
        return finger_data
```

### Option 2: Complementary Nodes Approach
**Use Case:** When you need complex post-processing or different control strategies

**Architecture:**
```
Hand Pose Node → Finger Processor Node → Servo Controller Node
       ↓              ↓                      ↓
   Raw Landmarks  Processed Angles    Servo Commands
```

## Recommended Implementation: Enhanced Hand Pose Node

### Enhanced Hand Pose Node Features

#### 1. Finger Landmark Groups
```python
# MediaPipe Hand Landmark Indices
FINGER_LANDMARKS = {
    'thumb': [1, 2, 3, 4],      # Thumb joints
    'index': [5, 6, 7, 8],      # Index finger joints  
    'middle': [9, 10, 11, 12],  # Middle finger joints
    'ring': [13, 14, 15, 16],   # Ring finger joints
    'pinky': [17, 18, 19, 20]   # Pinky finger joints
}

WRIST_LANDMARK = 0  # Reference point
```

#### 2. Finger Angle Calculation
```python
def calculate_finger_bend(self, landmarks, finger_indices):
    """Calculate finger bend angle from landmarks."""
    wrist = landmarks[0]
    mcp = landmarks[finger_indices[0]]  # Metacarpophalangeal joint
    tip = landmarks[finger_indices[-1]]  # Fingertip
    
    # Vector calculations for bend angle
    vec_base = self.get_vector(wrist, mcp)
    vec_finger = self.get_vector(mcp, tip)
    
    # Calculate angle between vectors
    angle = self.calculate_angle(vec_base, vec_finger)
    return angle

def calculate_finger_curl(self, landmarks, finger_indices):
    """Calculate finger curl (how closed the finger is)."""
    # Distance-based calculation
    mcp = landmarks[finger_indices[0]]
    tip = landmarks[finger_indices[-1]]
    
    # Calculate curl as ratio of current to maximum distance
    current_dist = self.calculate_distance(mcp, tip)
    max_dist = self.get_max_finger_length(finger_indices)
    
    curl_ratio = 1.0 - (current_dist / max_dist)
    return np.clip(curl_ratio, 0.0, 1.0)
```

#### 3. Servo Position Mapping
```python
def map_to_servo_position(self, finger_curl, finger_name):
    """Map finger curl to servo position (0-4095 range)."""
    # Calibrated ranges for each finger
    servo_ranges = {
        'thumb': (1200, 2800),   # Thumb has different range
        'index': (1000, 3000),   
        'middle': (1000, 3000),
        'ring': (1000, 3000),
        'pinky': (1100, 2900)    # Pinky slightly different
    }
    
    min_pos, max_pos = servo_ranges[finger_name]
    servo_pos = min_pos + finger_curl * (max_pos - min_pos)
    
    return int(np.clip(servo_pos, 0, 4095))
```

### Data Flow Architecture

```
Camera Feed → Hand Pose Node → Multiple Outputs:
                    ↓
    ┌─────────────────────────────────────────┐
    │           Hand Pose Node                │
    │  ┌─────────────────────────────────┐    │
    │  │     MediaPipe Processing        │    │
    │  │  • Hand detection & tracking    │    │
    │  │  • 21 landmark extraction       │    │
    │  └─────────────────────────────────┘    │
    │               ↓                         │
    │  ┌─────────────────────────────────┐    │
    │  │   Finger Position Analysis      │    │
    │  │  • Bend angle calculation       │    │
    │  │  • Curl ratio determination     │    │
    │  │  • Servo position mapping       │    │
    │  └─────────────────────────────────┘    │
    └─────────────────────────────────────────┘
                    ↓
        ┌───────────────────────────┐
        │      Published Topics      │
        │                           │
        │ /perception/hand_pose     │ → UI Dashboard
        │ /perception/finger_angles │ → Analysis Tools
        │ /servo/finger_commands    │ → Servo Controller
        │ /perception/hand_confidence│ → System Monitor
        └───────────────────────────┘
```

## Corrected Architecture: Separate Servo Control

### Current Implementation Architecture

```
Camera Feed → Hand Pose Node → Finger Data → Servo Controller → Hardware Interface
                    ↓              ↓              ↓                    ↓
            Raw Landmarks  Finger Positions  Servo Commands     Physical Servos
```

**Data Flow:**
1. **Hand Pose Node** - Processes MediaPipe, publishes finger curl ratios and angles
2. **Finger Servo Controller** - Converts finger data to servo commands with safety/smoothing
3. **Servo Interface Node** - Hardware abstraction for Dynamixel communication

### Corrected Topic Structure

#### Hand Pose Node Publishes:
- `/perception/hand_pose` - Raw hand landmarks (existing)
- `/perception/finger_angles` - Finger bend angles in radians
- `/perception/finger_curl_ratios` - Finger curl ratios (0=open, 1=closed)
- `/perception/hand_confidence` - Detection confidence (existing)

#### Servo Controller Node:
**Subscribes to:**
- `/perception/finger_curl_ratios` - Input finger data
- `/emergency_stop` - Safety override
- `/mode` - Control mode switching

**Publishes:**
- `/servo/commands` - Direct servo position commands (0-4095)
- `/servo/status` - Controller status and health

#### Hardware Interface Node:
**Subscribes to:**
- `/servo/commands` - Position commands from controller

**Publishes:**
- `/servo/positions` - Current actual servo positions
- `/servo/hardware_status` - Hardware diagnostics

### Custom Message Definitions

```python
# Create custom_msgs/FingerStates.msg
Header header
string hand_label          # "Left" or "Right" 
float32 confidence        # Detection confidence
float32[] curl_ratios     # Finger curl ratios [0.0-1.0]
float32[] bend_angles     # Finger bend angles in radians
int32[] servo_positions   # Target servo positions [0-4095]
bool[] finger_detected    # Per-finger detection status
```

## Implementation Strategy

### Phase 1: Enhanced Hand Pose Node
```python
#!/usr/bin/env python3
"""
Enhanced Hand Pose Node with Finger Position Extraction
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32, String, Float32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge
import mediapipe as mp

class EnhancedHandPoseNode(Node):
    """
    Enhanced hand pose detection with finger position extraction.
    - MediaPipe hand tracking with 21 landmarks
    - Real-time finger angle and curl calculation
    - Direct servo position mapping
    - Multiple output topics for different use cases
    """
    
    def __init__(self):
        super().__init__('hand_pose_node')
        
        # ...existing initialization...
        
        # Additional publishers for finger data
        self.finger_angles_pub = self.create_publisher(
            Float32MultiArray, '/perception/finger_angles', 10)
        self.finger_curl_pub = self.create_publisher(
            Float32MultiArray, '/perception/finger_curl_ratios', 10)
        self.servo_commands_pub = self.create_publisher(
            Float32MultiArray, '/servo/finger_commands', 10)
        
        # Finger processing configuration
        self.finger_landmarks = {
            'thumb': [1, 2, 3, 4],
            'index': [5, 6, 7, 8], 
            'middle': [9, 10, 11, 12],
            'ring': [13, 14, 15, 16],
            'pinky': [17, 18, 19, 20]
        }
        
        # Servo calibration data (to be fine-tuned)
        self.servo_ranges = {
            'thumb': (1200, 2800),
            'index': (1000, 3000),
            'middle': (1000, 3000), 
            'ring': (1000, 3000),
            'pinky': (1100, 2900)
        }
        
        # Smoothing for servo commands
        self.prev_servo_positions = np.array([2048, 2048, 2048, 2048, 2048])
        self.smoothing_factor = 0.7
        
        self.get_logger().info('Enhanced hand pose node with finger extraction started')
    
    def extract_and_publish_hand_poses(self, results, image_shape):
        """Enhanced version with finger position extraction."""
        # ...existing pose array creation...
        
        if preferred_hand_landmarks:
            # Existing landmark processing
            confidence = preferred_confidence
            for landmark in preferred_hand_landmarks.landmark:
                pose = Pose()
                pose.position.x = landmark.x
                pose.position.y = landmark.y  
                pose.position.z = landmark.z
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)
            
            # NEW: Extract finger positions
            finger_data = self.extract_finger_positions(preferred_hand_landmarks)
            self.publish_finger_data(finger_data)
        
        # Publish existing data
        self.pose_pub.publish(pose_array)
        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.confidence_pub.publish(confidence_msg)
    
    def extract_finger_positions(self, hand_landmarks):
        """Extract finger positions and angles from hand landmarks."""
        landmarks = hand_landmarks.landmark
        finger_data = {}
        
        for finger_name, indices in self.finger_landmarks.items():
            curl_ratio = self.calculate_finger_curl(landmarks, indices)
            bend_angle = self.calculate_finger_bend(landmarks, indices)
            servo_pos = self.map_to_servo_position(curl_ratio, finger_name)
            
            finger_data[finger_name] = {
                'curl_ratio': curl_ratio,
                'bend_angle': bend_angle,
                'servo_position': servo_pos
            }
        
        return finger_data
    
    def calculate_finger_curl(self, landmarks, finger_indices):
        """Calculate how curled/closed a finger is (0=open, 1=closed)."""
        # Get key points
        mcp = landmarks[finger_indices[0]]  # Base joint
        tip = landmarks[finger_indices[-1]]  # Fingertip
        
        # Calculate current distance
        current_dist = np.sqrt(
            (tip.x - mcp.x)**2 + 
            (tip.y - mcp.y)**2 + 
            (tip.z - mcp.z)**2
        )
        
        # Estimate maximum distance (finger fully extended)
        # This is a simplified estimation - could be calibrated per user
        max_dist = 0.15  # Approximately 15cm for average finger
        
        # Calculate curl ratio
        curl_ratio = 1.0 - np.clip(current_dist / max_dist, 0.0, 1.0)
        return curl_ratio
    
    def calculate_finger_bend(self, landmarks, finger_indices):
        """Calculate finger bend angle in radians."""
        wrist = landmarks[0]
        mcp = landmarks[finger_indices[0]]
        tip = landmarks[finger_indices[-1]]
        
        # Vectors
        vec_base = np.array([mcp.x - wrist.x, mcp.y - wrist.y, mcp.z - wrist.z])
        vec_finger = np.array([tip.x - mcp.x, tip.y - mcp.y, tip.z - mcp.z])
        
        # Normalize
        vec_base = vec_base / (np.linalg.norm(vec_base) + 1e-8)
        vec_finger = vec_finger / (np.linalg.norm(vec_finger) + 1e-8)
        
        # Calculate angle
        dot_product = np.clip(np.dot(vec_base, vec_finger), -1.0, 1.0)
        angle = np.arccos(dot_product)
        
        return angle
    
    def map_to_servo_position(self, curl_ratio, finger_name):
        """Map curl ratio to servo position."""
        min_pos, max_pos = self.servo_ranges[finger_name]
        servo_pos = min_pos + curl_ratio * (max_pos - min_pos)
        return int(np.clip(servo_pos, 0, 4095))
    
    def publish_finger_data(self, finger_data):
        """Publish all finger-related data."""
        fingers = ['thumb', 'index', 'middle', 'ring', 'pinky']
        
        # Extract arrays
        curl_ratios = [finger_data[f]['curl_ratio'] for f in fingers]
        bend_angles = [finger_data[f]['bend_angle'] for f in fingers] 
        servo_positions = [finger_data[f]['servo_position'] for f in fingers]
        
        # Apply smoothing to servo positions
        servo_array = np.array(servo_positions)
        smoothed_servos = (self.smoothing_factor * self.prev_servo_positions + 
                          (1 - self.smoothing_factor) * servo_array)
        self.prev_servo_positions = smoothed_servos
        
        # Publish curl ratios
        curl_msg = Float32MultiArray()
        curl_msg.data = curl_ratios
        self.finger_curl_pub.publish(curl_msg)
        
        # Publish bend angles
        angle_msg = Float32MultiArray()
        angle_msg.data = bend_angles
        self.finger_angles_pub.publish(angle_msg)
        
        # Publish servo commands
        servo_msg = Float32MultiArray()
        servo_msg.data = smoothed_servos.tolist()
        self.servo_commands_pub.publish(servo_msg)
        
        # Debug logging
        self.get_logger().debug(f'Finger curls: {[f"{c:.2f}" for c in curl_ratios]}')
        self.get_logger().debug(f'Servo positions: {[int(s) for s in smoothed_servos]}')
```

## Integration with Existing System

### Minimal Changes Required

1. **Hand Pose Node**: Enhance existing node with finger extraction
2. **UI Dashboard**: Add finger state visualization
3. **New Servo Package**: Create dedicated servo control package

### Pose To Command Node Role

The pose_to_command_node remains complementary for:
- **High-level coordination**: Managing different control modes
- **Safety logic**: Implementing emergency stops and limits
- **Multi-modal control**: Switching between hand control, automation, etc.

```python
class PoseToCommandNode(Node):
    """
    Enhanced pose to command coordination.
    - Manages control modes and safety
    - Coordinates between different input sources
    - Implements high-level control logic
    """
    
    def __init__(self):
        # ...existing code...
        
        # Subscribe to finger commands from hand pose node
        self.finger_sub = self.create_subscription(
            Float32MultiArray, '/servo/finger_commands', 
            self.handle_finger_commands, 10)
        
        # Subscribe to other control inputs
        self.manual_sub = self.create_subscription(
            Float32MultiArray, '/manual/finger_commands',
            self.handle_manual_commands, 10)
    
    def handle_finger_commands(self, msg):
        """Process finger commands based on current mode."""
        if self.current_mode == "pose_tracking":
            # Pass through hand pose commands
            self.forward_to_servos(msg)
        elif self.current_mode == "manual":
            # Ignore hand pose, wait for manual commands
            pass
        elif self.current_mode == "autonomous":
            # Use autonomous control logic
            self.execute_autonomous_sequence()
    
    def forward_to_servos(self, finger_commands):
        """Forward commands to servo controller with safety checks."""
        # Apply safety limits
        safe_commands = self.apply_safety_limits(finger_commands.data)
        
        # Publish to servo controller
        servo_msg = Float32MultiArray()
        servo_msg.data = safe_commands
        self.servo_output_pub.publish(servo_msg)
```

## Calibration and Testing Strategy

### Calibration Process
1. **Range Calibration**: Record min/max servo positions for each finger
2. **User Calibration**: Adapt finger length estimates to user's hand
3. **Smoothing Tuning**: Adjust smoothing factor for responsiveness vs stability

### Testing Phases
1. **Finger Detection**: Verify accurate finger curl calculation
2. **Servo Mapping**: Test servo position mapping accuracy
3. **Real-time Performance**: Measure latency and smoothness
4. **User Testing**: Test with different hand sizes and movements

## Advantages of This Architecture

1. **Single Source of Truth**: Hand pose node processes all hand data
2. **Minimal Latency**: Direct processing eliminates pipeline delays
3. **Simplified Debugging**: All hand processing in one location
4. **Flexible Output**: Multiple topic outputs for different consumers
5. **Future Extensibility**: Easy to add new finger analysis features

## Conclusion

The enhanced hand pose node approach provides the most efficient and maintainable solution for finger position extraction. It leverages the existing MediaPipe integration while adding specialized finger analysis capabilities directly in the data source node.

This architecture maintains the modularity of the system while eliminating unnecessary data transformations and reducing latency for real-time servo control.
