#!/usr/bin/env python3
"""
Finger Servo Controller Node

Converts finger curl ratios from hand pose detection into servo commands
for Dynamixel XL330-288 servos. Handles calibration, safety limits, and
smooth servo control.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
import numpy as np
import time

class FingerServoControllerNode(Node):
    """
    Finger servo controller for robotic hand control.
    
    Subscribes to:
    - /perception/finger_curl_ratios - Finger curl data from hand pose
    - /ui/emergency_stop - Emergency stop signal
    - /control/mode - Control mode (pose_tracking, manual, autonomous)
    
    Publishes to:
    - /servo/commands - Direct servo position commands
    - /servo/status - Servo status and health information
    """
    
    def __init__(self):
        super().__init__('finger_servo_controller_node')
        
        # Control state
        self.control_mode = "idle"  # idle, pose_tracking, manual, autonomous
        self.emergency_stop = False
        self.servo_enabled = False
        
        # Finger configuration
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
        
        # Servo calibration - These should be calibrated for your specific setup
        self.servo_ranges = {
            'thumb': (1200, 2800),   # Min/max positions for thumb
            'index': (1000, 3000),   # Min/max positions for index
            'middle': (1000, 3000),  # Min/max positions for middle  
            'ring': (1000, 3000),    # Min/max positions for ring
            'pinky': (1100, 2900)    # Min/max positions for pinky
        }
        
        # Safety limits
        self.max_speed = 100        # Maximum servo speed
        self.max_position_change = 200  # Maximum position change per update
        
        # Servo state tracking
        self.current_positions = np.array([2048, 2048, 2048, 2048, 2048])  # Midpoint
        self.target_positions = np.array([2048, 2048, 2048, 2048, 2048])
        self.last_update_time = time.time()
        
        # Smoothing and filtering
        self.smoothing_factor = 0.8  # Higher = more smoothing
        self.position_filter = np.array([2048, 2048, 2048, 2048, 2048])
        
        # Subscribers
        self.finger_curl_sub = self.create_subscription(
            Float32MultiArray,
            '/perception/finger_curl_ratios',
            self.handle_finger_curl_data,
            10)
            
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.handle_emergency_stop,
            5)
            
        self.mode_sub = self.create_subscription(
            String,
            '/mode',
            self.handle_mode_change,
            5)
        
        # Publishers
        self.servo_commands_pub = self.create_publisher(
            Float32MultiArray,
            '/servo/commands',
            10)
            
        self.servo_status_pub = self.create_publisher(
            String,
            '/servo/status',
            10)
        
        # Control timer for regular servo updates
        self.control_timer = self.create_timer(0.05, self.update_servos)  # 20Hz control loop
        
        # Status timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Finger servo controller node started')
        self.get_logger().info(f'Servo ranges: {self.servo_ranges}')

    def handle_finger_curl_data(self, msg):
        """Process finger curl ratios and convert to servo targets."""
        if self.emergency_stop:
            self.get_logger().debug('Emergency stop active - ignoring finger data')
            return
            
        if self.control_mode != "pose_tracking":
            self.get_logger().debug(f'Not in pose tracking mode ({self.control_mode}) - ignoring finger data')
            return
            
        if len(msg.data) != 5:
            self.get_logger().warn(f'Expected 5 finger values, got {len(msg.data)}')
            return
        
        # Convert curl ratios to servo positions
        new_targets = []
        for i, finger_name in enumerate(self.finger_names):
            curl_ratio = np.clip(msg.data[i], 0.0, 1.0)  # Ensure valid range
            servo_pos = self.map_curl_to_servo_position(curl_ratio, finger_name)
            new_targets.append(servo_pos)
        
        # Apply smoothing to targets
        new_targets = np.array(new_targets)
        self.target_positions = (self.smoothing_factor * self.target_positions + 
                               (1 - self.smoothing_factor) * new_targets)
        
        self.get_logger().debug(f'Finger curls: {[f"{c:.2f}" for c in msg.data]}')
        self.get_logger().debug(f'Target positions: {[int(p) for p in self.target_positions]}')

    def map_curl_to_servo_position(self, curl_ratio, finger_name):
        """Map finger curl ratio to servo position."""
        min_pos, max_pos = self.servo_ranges[finger_name]
        servo_pos = min_pos + curl_ratio * (max_pos - min_pos)
        return int(np.clip(servo_pos, 0, 4095))

    def handle_emergency_stop(self, msg):
        """Handle emergency stop signal."""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop activated - servo control disabled')
            # Immediately stop all servo movement
            self.target_positions = self.current_positions.copy()
        else:
            self.get_logger().info('Emergency stop released - servo control enabled')

    def handle_mode_change(self, msg):
        """Handle control mode changes."""
        new_mode = msg.data
        if new_mode != self.control_mode:
            self.get_logger().info(f'Control mode changed from {self.control_mode} to {new_mode}')
            self.control_mode = new_mode
            
            # Reset to safe positions when changing modes
            if new_mode == "idle":
                # Move to neutral positions
                neutral_positions = np.array([2048, 2048, 2048, 2048, 2048])
                self.target_positions = neutral_positions

    def update_servos(self):
        """Main servo control loop - called at 20Hz."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if self.emergency_stop:
            # Don't move servos during emergency stop
            return
            
        # Apply safety limits to position changes
        position_diff = self.target_positions - self.current_positions
        max_change = self.max_position_change * dt * 20  # Scale by time and frequency
        
        # Limit the maximum change per update
        position_diff = np.clip(position_diff, -max_change, max_change)
        new_positions = self.current_positions + position_diff
        
        # Ensure positions are within valid servo range
        new_positions = np.clip(new_positions, 0, 4095)
        
        # Update current positions
        self.current_positions = new_positions
        self.last_update_time = current_time
        
        # Publish servo commands
        if self.control_mode in ["pose_tracking", "manual", "autonomous"]:
            self.publish_servo_commands(new_positions)

    def publish_servo_commands(self, positions):
        """Publish servo position commands."""
        servo_msg = Float32MultiArray()
        servo_msg.data = positions.tolist()
        self.servo_commands_pub.publish(servo_msg)
        
        self.get_logger().debug(f'Servo commands: {[int(p) for p in positions]}')

    def publish_status(self):
        """Publish periodic status information."""
        status_msg = String()
        status_info = {
            'mode': self.control_mode,
            'emergency_stop': self.emergency_stop,
            'positions': [int(p) for p in self.current_positions],
            'targets': [int(p) for p in self.target_positions]
        }
        status_msg.data = str(status_info)
        self.servo_status_pub.publish(status_msg)

    def set_manual_positions(self, positions):
        """Set servo positions manually (for testing/calibration)."""
        if len(positions) == 5:
            self.target_positions = np.array(positions)
            self.get_logger().info(f'Manual servo positions set: {positions}')

def main(args=None):
    rclpy.init(args=args)
    node = FingerServoControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Servo controller node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
