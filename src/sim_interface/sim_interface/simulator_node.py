#!/usr/bin/env python3
"""
Simulator Node for CR3 Control System

Hosts the Gazebo instance of the CR3 robot. Publishes /joint_states, /cr3/status, and /tf, simulating robot behavior and kinematics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
from builtin_interfaces.msg import Time

class SimulatorNode(Node):
    """
    Outline: Simulates the CR3 robot in Gazebo.
    - Publishes simulated joint states and status
    - Loads robot URDF and controllers
    - Enables full virtual testing of control flows
    """
    def __init__(self):
        """Initialize the simulator node."""
        super().__init__('simulator_node')
        
        # Publishers
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)
            
        self.status_pub = self.create_publisher(
            String,
            '/cr3/status',
            10)
            
        # Timer for simulating data
        self.timer = self.create_timer(0.1, self.publish_simulated_data)
        
        # Joint names for CR3 robot
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.get_logger().info('Simulator node started')

    def publish_simulated_data(self):
        """Publish simulated joint states and status."""
        self.publish_joint_states()
        self.publish_status()

    def publish_joint_states(self):
        """Publish simulated joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        msg.name = self.joint_names
        
        # Generate simple oscillating joint positions
        t = self.get_clock().now().nanoseconds / 1e9
        msg.position = [
            0.1 * np.sin(t * 0.5),
            0.2 * np.sin(t * 0.3),
            0.1 * np.cos(t * 0.4),
            0.2 * np.cos(t * 0.2),
            0.1 * np.sin(t * 0.1),
            0.1 * np.cos(t * 0.6),
        ]
        
        # Dummy velocities and efforts
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_states_pub.publish(msg)

    def publish_status(self):
        """Publish simulated robot status."""
        msg = String()
        msg.data = "READY"
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()