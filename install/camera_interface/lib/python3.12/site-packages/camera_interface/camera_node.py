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
        
        # CV bridge for converting between OpenCV and ROS images
        self.cv_bridge = CvBridge()
        
        # Try to initialize camera
        self.cap = None
        self.use_camera = self.init_camera()
        
        # Timer for capturing and publishing
        self.timer = self.create_timer(0.033, self.capture_and_publish)  # ~30 FPS
        
        if self.use_camera:
            self.get_logger().info('Camera node started with real camera')
        else:
            self.get_logger().error('Camera node started but NO CAMERA DETECTED - no images will be published')

    def init_camera(self):
        """Initialize camera capture."""
        try:
            # Try to open camera device 0
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().warn('Cannot open camera device 0, trying device 1')
                self.cap = cv2.VideoCapture(1)
                
            if not self.cap.isOpened():
                self.get_logger().warn('No camera devices found, using dummy images')
                return False
                
            # Set camera properties for better performance
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to minimize latency
            
            # Additional performance settings
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))  # Use MJPEG for better performance
            
            # Test capture
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Camera opened but cannot capture frames, using dummy images')
                self.cap.release()
                return False
                
            self.get_logger().info(f'Camera initialized: {frame.shape}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error initializing camera: {e}')
            return False

    def capture_image(self):
        """Capture image from camera hardware."""
        if self.use_camera and self.cap is not None:
            try:
                ret, frame = self.cap.read()
                if ret:
                    return frame
                else:
                    self.get_logger().error('Failed to capture frame from camera')
                    return None
            except Exception as e:
                self.get_logger().error(f'Error capturing from camera: {e}')
                return None
        else:
            self.get_logger().error('No camera available')
            return None

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
        if image is not None:
            self.publish_image(image)

    def cleanup(self):
        """Clean up camera resources."""
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info('Camera resources released')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()