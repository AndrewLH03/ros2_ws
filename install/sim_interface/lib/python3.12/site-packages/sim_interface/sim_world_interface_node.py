#!/usr/bin/env python3
"""
Sim World Interface Node for CR3 Control System

Adds dynamic elements to the Gazebo world, such as objects, goals, or obstacles. Publishes /sim/obstacles and /sim/targets.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3, Pose
from std_msgs.msg import ColorRGBA
import random
import math

class SimWorldInterfaceNode(Node):
    """
    Outline: Manages dynamic elements in the Gazebo simulation world.
    - Spawns models and obstacles
    - Publishes simulation targets and obstacles
    - Sets up test scenarios for planning and control
    """
    def __init__(self):
        """Initialize the sim world interface node."""
        super().__init__('sim_world_interface_node')
        
        # Publishers
        self.obstacles_pub = self.create_publisher(
            MarkerArray,
            '/sim/obstacles',
            10)
            
        self.targets_pub = self.create_publisher(
            MarkerArray,
            '/sim/targets',
            10)
            
        # Timer for publishing
        self.timer = self.create_timer(1.0, self.publish_all)
        
        # Counters for unique IDs
        self.obstacle_id = 0
        self.target_id = 0
        
        self.get_logger().info('Sim world interface node started')

    def publish_all(self):
        """Publish all simulation elements."""
        self.publish_obstacles()
        self.publish_targets()

    def publish_obstacles(self):
        """Publish obstacle information to /sim/obstacles."""
        marker_array = MarkerArray()
        
        # Create 3 random obstacles
        for i in range(3):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = self.obstacle_id
            self.obstacle_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Random position in workspace
            marker.pose.position.x = 0.5 + 0.2 * math.sin(self.get_clock().now().nanoseconds / 1e9 + i)
            marker.pose.position.y = 0.3 * math.cos(self.get_clock().now().nanoseconds / 1e9 + i)
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            
            # Size
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Red color for obstacles
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        self.obstacles_pub.publish(marker_array)

    def publish_targets(self):
        """Publish target information to /sim/targets."""
        marker_array = MarkerArray()
        
        # Create a target
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.target_id
        self.target_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Random position for target
        marker.pose.position.x = 0.6
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Green color for target
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        
        marker_array.markers.append(marker)
        
        self.targets_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SimWorldInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()