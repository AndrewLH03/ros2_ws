#!/usr/bin/env python3
"""
Mode Switcher Node for CR3 Control System

Lets users toggle between control modes (manual, pose_tracking, autonomous). Publishes /mode, which controls how pose_to_command_node responds.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter as rclpy_Parameter
import time

class ModeSwitcherNode(Node):
    """
    Allows toggling between control modes.
    - Publishes /mode topic
    - Handles user input (CLI, GUI, or parameter)
    - Ensures state machine consistency and debounce
    """
    def __init__(self):
        """Initialize the mode switcher node."""
        super().__init__('mode_switcher_node')
        
        # Define available modes
        self.modes = ["manual", "pose_tracking", "autonomous"]
        self.current_mode_index = 1  # Default to pose_tracking
        self.last_mode_change_time = time.time()
        self.debounce_time = 1.0  # 1 second debounce
        
        # Publisher for mode
        self.mode_pub = self.create_publisher(
            String,
            '/mode',
            10)
            
        # Parameter for changing mode
        self.declare_parameter('control_mode', 'pose_tracking')
        
        # Create a timer to check for parameter changes and publish mode
        self.timer = self.create_timer(0.5, self.check_and_publish_mode)
        
        # Create a service to cycle through modes (for testing)
        self.cycle_mode_service = self.create_service(
            SetParameters,
            '~/cycle_mode',
            self.cycle_mode_callback)
        
        self.get_logger().info('Mode switcher node started')
        
    def check_and_publish_mode(self):
        """Check current parameter value and publish mode."""
        param_value = self.get_parameter('control_mode').value
        if param_value in self.modes:
            self.publish_mode(param_value)
        else:
            self.get_logger().warn(f'Invalid mode: {param_value}')
        
    def handle_user_input(self, input_data):
        """Handle user input to change mode."""
        # This would typically come from a GUI or console input
        # For the minimal version, we'll use a parameter
        if time.time() - self.last_mode_change_time < self.debounce_time:
            self.get_logger().info('Debouncing mode change')
            return
            
        if input_data in self.modes:
            self.last_mode_change_time = time.time()
            self.get_parameter('control_mode').value = input_data
            self.publish_mode(input_data)
        else:
            self.get_logger().warn(f'Invalid mode: {input_data}')

    def publish_mode(self, mode):
        """Publish the selected mode to /mode."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Mode set to: {mode}')
        
    def cycle_mode_callback(self, request, response):
        """Service callback to cycle through available modes."""
        self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)
        new_mode = self.modes[self.current_mode_index]
        
        # Update parameter
        param = rclpy_Parameter('control_mode', Parameter.PARAMETER_STRING, new_mode)
        self.set_parameters([param])
        
        # Publish new mode
        self.publish_mode(new_mode)
        
        # Set response parameters
        response.results = []
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()