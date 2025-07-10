#!/usr/bin/env python3
"""
Teleop Node for CR3 Control System

Manual control fallback for issuing robot commands using keyboard, joystick, or touchscreen input. Publishes /cr3/target_pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
import numpy as np
import math

class TeleopNode(Node):
    """
    Outline: Manual teleoperation node for the CR3 robot.
    - Handles device input (joystick, keyboard)
    - Maps UI input to robot poses or joint increments
    - Publishes target poses for manual control
    """
    def __init__(self):
        """Initialize the teleop node."""
        super().__init__('teleop_node')
        
        # Publisher for target pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10)
            
        # Create a timer to generate demo motion (for rqt_graph visualization)
        self.timer = self.create_timer(0.5, self.publish_demo_pose)
        
        # Keeping track of demo position
        self.demo_time = 0.0
        
        # Base pose for demo motion
        self.base_pose = {
            'x': 0.5,
            'y': 0.0,
            'z': 0.4
        }
        
        self.get_logger().info('Teleop node started')

    def handle_input(self, input_data):
        """Handle manual input from devices."""
        # In a real implementation, this would process joystick/keyboard input
        # For minimal demo, we just use a timer-based approach
        target_pose = self.compute_target_pose(input_data)
        self.publish_pose_command(target_pose)

    def compute_target_pose(self, input_data):
        """Compute the target pose based on input data."""
        # Simple implementation for demo - returns a fixed pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.3
        
        # Default orientation - gripper pointing down
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.707
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.707
        
        return pose

    def publish_pose_command(self, target_pose):
        """Publish the computed pose command to /target_pose."""
        self.pose_pub.publish(target_pose)
        self.get_logger().debug(f"Published target pose: x={target_pose.pose.position.x}, y={target_pose.pose.position.y}, z={target_pose.pose.position.z}")
        
    def publish_demo_pose(self):
        """Publish a demo pose that moves in a circle."""
        self.demo_time += 0.1
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        
        # Circle motion in X-Y plane
        radius = 0.1
        pose.pose.position.x = self.base_pose['x'] + radius * math.cos(self.demo_time)
        pose.pose.position.y = self.base_pose['y'] + radius * math.sin(self.demo_time)
        pose.pose.position.z = self.base_pose['z'] + 0.05 * math.sin(self.demo_time * 2)
        
        # Default orientation - gripper pointing down
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.707
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.707
        
        self.publish_pose_command(pose)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()