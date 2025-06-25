#!/usr/bin/env python3
"""
UI Dashboard Node for CR3 Control System

Outline-only version. Subscribes to all diagnostic topics and displays them in a human-friendly format. May publish to /mode or offer a web interface. Serves as the command center for live debugging, monitoring, and user-triggered commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import time

class UIDashboardNode(Node):
    """
    Outline: UI dashboard for monitoring and control.
    - Subscribes to diagnostic/status topics
    - Displays information in a user-friendly format
    - Publishes commands or mode changes
    - May provide a web interface
    """
    def __init__(self):
        """Initialize the UI dashboard node."""
        super().__init__('ui_dashboard_node')
        
        # Store status information
        self.status_data = {}
        
        # Subscribe to topics
        self.subscribe_to_topics()
        
        # Publishers
        self.mode_pub = self.create_publisher(
            String,
            '/mode',
            10)
            
        self.emergency_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10)
            
        # Timer for demo updates
        self.timer = self.create_timer(5.0, self.publish_demo_commands)
        
        # Mode cycle for demo
        self.modes = ["manual", "pose_tracking", "autonomous"]
        self.current_mode_index = 0
        
        self.get_logger().info('UI dashboard node started')

    def subscribe_to_topics(self):
        """Subscribe to diagnostic and status topics."""
        # Robot status
        self.create_subscription(
            String,
            '/cr3/status',
            self.handle_robot_status,
            10)
            
        # Joint states
        self.create_subscription(
            JointState,
            '/joint_states',
            self.handle_joint_states,
            10)
            
        # Target pose
        self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.handle_target_pose,
            10)
            
        # Health status
        self.create_subscription(
            String,
            '/diagnostics/health',
            self.handle_health,
            10)
            
        # Log messages
        self.create_subscription(
            String,
            '/log/command_sequence',
            self.handle_log,
            10)

    def handle_robot_status(self, msg):
        """Handle robot status messages."""
        self.status_data['robot_status'] = msg.data
        self.display_status('Robot status: ' + msg.data)

    def handle_joint_states(self, msg):
        """Handle joint state messages."""
        # Store only latest joint positions
        if len(msg.position) > 0:
            self.status_data['joint_positions'] = msg.position
            # Only log at debug level to avoid flooding
            self.get_logger().debug(f'Received joint positions: {msg.position}')

    def handle_target_pose(self, msg):
        """Handle target pose messages."""
        self.status_data['target_pose'] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        self.display_status(f'Target pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}')

    def handle_health(self, msg):
        """Handle health status messages."""
        self.status_data['health'] = msg.data
        self.display_status('Health: ' + msg.data)

    def handle_log(self, msg):
        """Handle log messages."""
        self.display_status('Log: ' + msg.data)

    def display_status(self, status):
        """Display system status in the UI."""
        # For minimal implementation, just log to console
        self.get_logger().info(status)

    def publish_mode_change(self, mode):
        """Publish a mode change command."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.display_status(f'Changed mode to: {mode}')

    def publish_emergency_stop(self, stop=True):
        """Publish an emergency stop command."""
        msg = Bool()
        msg.data = stop
        self.emergency_pub.publish(msg)
        self.display_status(f'Emergency stop: {stop}')

    def publish_demo_commands(self):
        """Publish demo commands for visualization."""
        # Cycle through modes
        next_mode = self.modes[self.current_mode_index]
        self.publish_mode_change(next_mode)
        
        self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)

def main(args=None):
    rclpy.init(args=args)
    node = UIDashboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()