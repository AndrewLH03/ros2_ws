#!/usr/bin/env python3
"""
Camera Node for CR3 Control System

Captures and publishes image data from the camera.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    """
    Captures and publishes image data.
    - Captures images from camera hardware
    - Publishes Image messages for perception
    """
    def __init__(self):
        """Initialize the camera node."""
        super().__init__('camera_node')
        
        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10)
            
        # Timer for simulating camera captures
        self.timer = self.create_timer(0.1, self.capture_and_publish)
        
        # CV bridge for converting between OpenCV and ROS images
        self.cv_bridge = CvBridge()
        
        self.get_logger().info('Camera node started')

    def capture_image(self):
        """Capture image from camera hardware (simulated)."""
        # Create a simple dummy image (grayscale gradient)
        height, width = 480, 640
        dummy_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Create a gradient pattern
        for y in range(height):
            for x in range(width):
                dummy_image[y, x] = [
                    int(x * 255 / width),
                    int(y * 255 / height),
                    128
                ]
        
        return dummy_image

    def publish_image(self, image):
        """Publish captured image data."""
        # Convert OpenCV image to ROS message
        img_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_frame"
        
        # Publish the image
        self.image_pub.publish(img_msg)
        
    def capture_and_publish(self):
        """Capture an image and publish it."""
        image = self.capture_image()
        self.publish_image(image)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

