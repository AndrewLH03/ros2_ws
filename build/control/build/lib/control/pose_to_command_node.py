#!/usr/bin/env python3
"""
Pose to Command Node for CR3 Control System - Minimal

Simplified stub for mode handling and potential future command generation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PoseToCommandNode(Node):
    """
    Minimal pose to command node.
    - Handles control mode changes
    - Framework for future robot command generation
    """
    def __init__(self):
        """Initialize the pose to command node."""
        super().__init__('pose_to_command_node')
        
        # Subscribe to mode changes
        self.mode_sub = self.create_subscription(
            String,
            '/mode',
            self.handle_mode_change,
            10)
            
        # Current mode
        self.current_mode = "pose_tracking"  # Default mode
        
        self.get_logger().info('Pose to command node started (minimal mode)')

    def handle_mode_change(self, mode_msg):
        """Handle changes in control mode."""
        self.current_mode = mode_msg.data
        self.get_logger().info(f'Mode changed to: {self.current_mode}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseToCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()