#!/usr/bin/env python3
"""
Body Pose Node for CR3 Control System

Detects and publishes human body pose information for downstream processing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose

class BodyPoseNode(Node):
    """
    Detects and publishes human body pose.
    - Subscribes to camera/image topics
    - Runs pose estimation algorithms
    - Publishes body pose data
    """
    def __init__(self):
        """Initialize the body pose node."""
        super().__init__('body_pose_node')
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            10)
            
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/body_pose_raw',
            10)
            
        # Heartbeat timer - publishes dummy data to make topics visible
        self.timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.get_logger().info('Body pose node started')

    def process_image(self, image):
        """Process incoming image and estimate body pose."""
        # Minimal implementation - just log receipt
        self.get_logger().debug('Received image')
        
        # In a real implementation, this would process the image
        # and extract body pose information

    def publish_body_pose(self, pose):
        """Publish estimated body pose."""
        self.pose_pub.publish(pose)
        
    def publish_heartbeat(self):
        """Publish dummy pose data to make topic visible in rqt_graph"""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"
        
        # Add a single pose 
        dummy_pose = Pose()
        dummy_pose.position.x = 0.5
        dummy_pose.position.y = 0.5
        dummy_pose.position.z = 1.0
        dummy_pose.orientation.w = 1.0
        msg.poses.append(dummy_pose)
        
        self.publish_body_pose(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BodyPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

