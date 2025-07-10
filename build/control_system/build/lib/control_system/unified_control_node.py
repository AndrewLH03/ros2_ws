#!/usr/bin/env python3
"""
Unified Control Node

Central control node that handles all control types:
- Manual control (digital input)
- Perception control (camera-based)
- Future control types can be added here

This node acts as a coordinator and router for different control modes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
import numpy as np


class UnifiedControlNode(Node):
    """
    Unified control node that manages all control types.
    
    Subscribes to:
    - /mode - Current control mode
    - /emergency_stop - Emergency stop signal
    - /perception/finger_curl_ratios - Finger data from perception/manual
    - /camera/image_raw - Camera feed (when in perception mode)
    
    Publishes to:
    - /servo/commands - Servo position commands
    - /control/status - Control system status
    """
    
    def __init__(self):
        super().__init__('unified_control_node')
        
        # Control state
        self.current_mode = "manual"  # Default to manual
        self.emergency_stop = False
        self.control_active = True
        
        # Finger configuration
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
        self.current_finger_curls = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Servo configuration (XL330-288 range: 0-4095)
        self.servo_ranges = [
            (1200, 2800),  # Thumb
            (1000, 3000),  # Index
            (1000, 3000),  # Middle
            (1000, 3000),  # Ring
            (1100, 2900),  # Pinky
        ]
        
        # Subscribers
        self.mode_sub = self.create_subscription(
            String, '/mode', self.handle_mode_change, 10)
            
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.handle_emergency_stop, 10)
            
        self.finger_curl_sub = self.create_subscription(
            Float32MultiArray, '/perception/finger_curl_ratios', 
            self.handle_finger_curls, 10)
        
        # Publishers
        self.servo_commands_pub = self.create_publisher(
            Float32MultiArray, '/servo/commands', 10)
            
        self.control_status_pub = self.create_publisher(
            String, '/control/status', 10)
        
        # Control timer - process servo commands at 20Hz
        self.control_timer = self.create_timer(0.05, self.process_control_commands)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f'Unified control node started in {self.current_mode} mode')
    
    def handle_mode_change(self, msg):
        """Handle control mode changes."""
        new_mode = msg.data
        if new_mode != self.current_mode:
            self.get_logger().info(f'Control mode changed: {self.current_mode} -> {new_mode}')
            self.current_mode = new_mode
            
            if new_mode == "manual":
                self.get_logger().info('🎮 Manual control mode activated')
            elif new_mode == "perception":
                self.get_logger().info('👁️ Perception control mode activated')
            else:
                self.get_logger().warn(f'Unknown control mode: {new_mode}')
    
    def handle_emergency_stop(self, msg):
        """Handle emergency stop signal."""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('🛑 Emergency stop activated - all control disabled')
            # Reset to safe positions
            self.current_finger_curls = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            self.get_logger().info('✅ Emergency stop released - control enabled')
    
    def handle_finger_curls(self, msg):
        """Handle incoming finger curl ratios from perception or manual control."""
        if len(msg.data) == 5:
            self.current_finger_curls = np.array(msg.data)
            self.get_logger().debug(f'Finger curls updated: {[f"{c:.2f}" for c in self.current_finger_curls]}')
        else:
            self.get_logger().warn(f'Invalid finger curl data length: {len(msg.data)}')
    
    def curl_to_servo_position(self, curl_ratio, finger_idx):
        """Convert finger curl ratio (0-1) to servo position (0-4095)."""
        if finger_idx >= len(self.servo_ranges):
            return 2048  # Default middle position
            
        min_pos, max_pos = self.servo_ranges[finger_idx]
        curl_ratio = np.clip(curl_ratio, 0.0, 1.0)
        servo_position = min_pos + curl_ratio * (max_pos - min_pos)
        return int(np.clip(servo_position, 0, 4095))
    
    def process_control_commands(self):
        """Main control processing loop."""
        if not self.control_active or self.emergency_stop:
            return
        
        # Convert finger curls to servo commands
        servo_commands = []
        for i, curl in enumerate(self.current_finger_curls):
            servo_pos = self.curl_to_servo_position(curl, i)
            servo_commands.append(float(servo_pos))
        
        # Publish servo commands
        servo_msg = Float32MultiArray()
        servo_msg.data = servo_commands
        self.servo_commands_pub.publish(servo_msg)
        
        # Log occasionally for debugging
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # ~1 Hz logging
            self.get_logger().debug(f'Servo commands: {[int(p) for p in servo_commands]}')
    
    def publish_status(self):
        """Publish control system status."""
        status_info = {
            'mode': self.current_mode,
            'emergency_stop': self.emergency_stop,
            'control_active': self.control_active,
            'finger_curls': [f"{c:.2f}" for c in self.current_finger_curls],
            'node_type': 'unified_control'
        }
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.control_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Unified control node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
