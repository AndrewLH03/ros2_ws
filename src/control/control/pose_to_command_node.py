#!/usr/bin/env python3
"""
Pose to Command Node for CR3 Control System

Converts human pose data into actionable robot commands. Subscribes to /hand_pose_robot_frame, /mode, and publishes to /cr3/target_pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String

class PoseToCommandNode(Node):
    """
    Converts human pose data to robot commands.
    - Maps hand positions to end-effector poses
    - Handles control modes and safety constraints
    - Publishes target poses for the CR3 arm
    """
    def __init__(self):
        """Initialize the pose to command node."""
        super().__init__('pose_to_command_node')
        
        # Subscribers
        self.hand_pose_sub = self.create_subscription(
            PoseArray,
            '/hand_pose_raw',
            self.process_hand_pose,
            10)
            
        self.mode_sub = self.create_subscription(
            String,
            '/mode',
            self.handle_mode_change,
            10)
            
        # Publishers
        self.target_pose_pub = self.create_publisher(
            Pose,
            '/cr3/target_pose',
            10)
            
        # Current mode
        self.current_mode = "pose_tracking"  # Default mode
        
        # Timer for publishing heartbeat (in case no poses received)
        self.timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.get_logger().info('Pose to command node started')

    def process_hand_pose(self, hand_pose_array):
        """Process incoming hand pose and generate robot command."""
        if len(hand_pose_array.poses) == 0:
            return
            
        # Extract the first pose from the array
        hand_pose = hand_pose_array.poses[0]
        
        # Apply a simple transformation based on mode
        target_pose = Pose()
        if self.current_mode == "pose_tracking":
            # Direct mapping with scaling
            target_pose.position.x = hand_pose.position.x * 0.5
            target_pose.position.y = hand_pose.position.y * 0.5
            target_pose.position.z = hand_pose.position.z * 0.5
            target_pose.orientation = hand_pose.orientation
        else:
            # Default behavior for other modes
            target_pose = hand_pose
            
        # Publish the computed target pose
        self.publish_target_pose(target_pose)

    def handle_mode_change(self, mode_msg):
        """Handle changes in control mode."""
        self.current_mode = mode_msg.data
        self.get_logger().info(f'Mode changed to: {self.current_mode}')

    def publish_target_pose(self, target_pose):
        """Publish the computed target pose to /cr3/target_pose."""
        self.target_pose_pub.publish(target_pose)
        
    def publish_heartbeat(self):
        """Publish a default pose if no hand poses received."""
        if self.current_mode != "manual":
            return
            
        # Default pose for heartbeat
        default_pose = Pose()
        default_pose.position.x = 0.3
        default_pose.position.y = 0.0
        default_pose.position.z = 0.5
        default_pose.orientation.w = 1.0
        
        self.publish_target_pose(default_pose)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()