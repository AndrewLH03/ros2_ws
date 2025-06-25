#!/usr/bin/env python3
"""
Hand Pose Node for CR3 Control System

Detects and publishes human hand pose information for downstream processing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose

class HandPoseNode(Node):
    """
    Detects and publishes human hand pose.
    - Subscribes to camera/image topics
    - Runs hand pose estimation algorithms
    - Publishes hand pose data
    """
    def __init__(self):
        """Initialize the hand pose node."""
        super().__init__('hand_pose_node')
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            10)
            
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/hand_pose_raw',
            10)
            
        # Heartbeat timer - publishes dummy data to make topics visible
        self.timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.get_logger().info('Hand pose node started')

    def process_image(self, image):
        """Process incoming image and estimate hand pose."""
        # Minimal implementation - just log receipt
        self.get_logger().debug('Received image')
        
        # In a real implementation, this would process the image
        # and extract hand pose information

    def publish_hand_pose(self, pose):
        """Publish estimated hand pose."""
        self.pose_pub.publish(pose)
        
    def publish_heartbeat(self):
        """Publish dummy pose data to make topic visible in rqt_graph"""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        
        # Add a single pose representing a hand
        dummy_pose = Pose()
        dummy_pose.position.x = 0.3
        dummy_pose.position.y = 0.2
        dummy_pose.position.z = 0.8
        dummy_pose.orientation.w = 1.0
        msg.poses.append(dummy_pose)
        
        self.publish_hand_pose(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

