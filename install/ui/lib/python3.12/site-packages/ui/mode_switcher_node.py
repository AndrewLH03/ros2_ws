#!/usr/bin/env python3
"""
Mode Switcher Node for CR3 Control System - Simplified

Basic mode switching between control modes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ModeSwitcherNode(Node):
    """
    Simple mode switching functionality.
    - Publishes /mode topic
    - Handles basic mode cycling
    """
    def __init__(self):
        """Initialize the mode switcher node."""
        super().__init__('mode_switcher_node')
        
        # Define available modes
        self.modes = ["manual", "pose_tracking", "vector_control", "autonomous"]
        self.current_mode_index = 1  # Default to pose_tracking
        
        # Publisher for mode changes
        self.mode_pub = self.create_publisher(
            String,
            '/mode',
            10)
        
        # Subscribe to UI commands for mode switching
        self.ui_command_sub = self.create_subscription(
            String,
            '/ui/command',
            self.handle_ui_commands,
            10)
        
        # Timer to periodically publish current mode
        self.timer = self.create_timer(2.0, self.publish_current_mode)
        
        self.get_logger().info('Mode switcher node started')
        self.publish_current_mode()

    def handle_ui_commands(self, msg):
        """Handle commands from the UI dashboard."""
        command = msg.data
        if command.startswith('set_mode:'):
            mode = command.split(':')[1]
            if mode in self.modes:
                self.set_mode(mode)
        elif command == 'cycle_mode':
            self.cycle_mode()
        
    def cycle_mode(self):
        """Cycle through available modes."""
        self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)
        new_mode = self.modes[self.current_mode_index]
        self.set_mode(new_mode)

    def set_mode(self, mode):
        """Set the current mode."""
        if mode in self.modes:
            self.current_mode_index = self.modes.index(mode)
            self.publish_mode(mode)
        else:
            self.get_logger().warn(f'Invalid mode: {mode}')

    def publish_mode(self, mode):
        """Publish the selected mode to /mode."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Mode set to: {mode}')
        
    def publish_current_mode(self):
        """Publish current mode periodically."""
        current_mode = self.modes[self.current_mode_index]
        self.publish_mode(current_mode)

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
