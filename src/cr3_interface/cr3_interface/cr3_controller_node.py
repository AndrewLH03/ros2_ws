#!/usr/bin/env python3
"""
CR3 Controller Node for CR3 Control System

Main communication interface to the physical CR3 robot. Subscribes to /cr3/target_pose and /cr3/emergency_stop, publishes /joint_states, /cr3/status, and /tf.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
import math
import numpy as np
import tf2_ros

class CR3ControllerNode(Node):
    """
    Main interface to the CR3 robot hardware.
    - Wraps robot SDK and exposes action servers
    - Handles pose and emergency stop commands
    - Publishes joint states and status
    """
    def __init__(self):
        """Initialize the CR3 controller node."""
        super().__init__('cr3_controller_node')
        
        # Subscribers
        self.target_pose_sub = self.create_subscription(
            Pose,
            '/cr3/target_pose',
            self.send_pose_to_robot,
            10)
            
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/cr3/emergency_stop',
            self.handle_emergency_stop,
            10)
            
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)
            
        self.status_pub = self.create_publisher(
            String,
            '/cr3/status',
            10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Robot state (simplified)
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 joints
        self.robot_status = "idle"
        
        # Timers for publishing state
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        self.status_timer = self.create_timer(1.0, self.monitor_status)
        self.tf_timer = self.create_timer(0.1, self.publish_tf_tree)
        
        self.get_logger().info('CR3 controller node started')

    def send_pose_to_robot(self, pose: Pose):
        """Send a pose command to the robot hardware."""
        # In a real implementation, this would use inverse kinematics and send commands
        # For the minimal version, we'll just update a simulated robot state
        
        # Simple mapping from Cartesian pose to joint angles (very simplified)
        # In reality, this would use proper inverse kinematics
        self.joint_positions[0] = math.atan2(pose.position.y, pose.position.x)  # Base rotation
        self.joint_positions[1] = 0.5 * pose.position.z  # Shoulder
        self.joint_positions[2] = 0.5 * math.sqrt(pose.position.x**2 + pose.position.y**2)  # Elbow
        self.joint_positions[3] = 0.2  # Wrist 1
        self.joint_positions[4] = 0.3  # Wrist 2
        self.joint_positions[5] = 0.1  # Wrist 3
        
        self.robot_status = "moving"
        self.get_logger().debug(f'Moving to pose: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}')

    def handle_emergency_stop(self, msg: Bool):
        """Handle emergency stop command."""
        if msg.data:
            self.robot_status = "emergency_stop"
            self.get_logger().warn('Emergency stop activated')

    def monitor_status(self):
        """Monitor and publish robot status."""
        status_msg = String()
        status_msg.data = self.robot_status
        self.status_pub.publish(status_msg)
        
        # Simulate returning to idle after movement
        if self.robot_status == "moving":
            self.robot_status = "idle"

    def publish_joint_states(self):
        """Publish current joint states."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [f'joint{i+1}' for i in range(6)]
        joint_state.position = self.joint_positions
        joint_state.velocity = [0.0] * 6  # No velocity in simplified model
        joint_state.effort = [0.0] * 6    # No effort in simplified model
        
        self.joint_state_pub.publish(joint_state)

    def publish_tf_tree(self):
        """Publish the robot's TF tree."""
        # This is a simplified TF tree for visualization
        # In a real implementation, this would use forward kinematics to compute transforms
        
        # Base to world transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'cr3_base'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # End effector to base transform (simplified)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'cr3_base'
        t.child_frame_id = 'cr3_end_effector'
        
        # Simplified forward kinematics (very rough approximation)
        x = 0.5 * math.cos(self.joint_positions[0])
        y = 0.5 * math.sin(self.joint_positions[0])
        z = 0.5 + 0.3 * self.joint_positions[1]
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CR3ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()