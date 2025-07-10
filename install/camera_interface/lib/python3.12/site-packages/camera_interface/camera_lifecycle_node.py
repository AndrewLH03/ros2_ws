#!/usr/bin/env python3
"""
Camera Lifecycle Node for CR3 Control System

Captures and publishes image data from the camera with lifecycle management.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import LifecyclePublisher
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class CameraLifecycleNode(LifecycleNode):
    """
    Captures and publishes image data with lifecycle management.
    - Configures camera hardware when configured
    - Starts/stops capture when activated/deactivated
    - Publishes Image messages for perception only when active
    """
    
    def __init__(self):
        """Initialize the camera lifecycle node."""
        super().__init__('camera_node')
        
        # Initialize variables (but don't create resources yet)
        self.image_pub = None
        self.cv_bridge = None
        self.cap = None
        self.timer = None
        self.use_camera = False
        
        self.get_logger().info('Camera lifecycle node initialized')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the camera - initialize hardware and publishers."""
        try:
            self.get_logger().info('Configuring camera...')
            
            # Initialize CV bridge
            self.cv_bridge = CvBridge()
            
            # Create lifecycle publisher
            self.image_pub = self.create_lifecycle_publisher(
                Image, '/camera/image_raw', 10)
            
            # Initialize camera hardware
            self.use_camera = self.init_camera()
            
            if self.use_camera:
                self.get_logger().info('Camera configured successfully with hardware')
            else:
                self.get_logger().warn('Camera configured but no hardware detected - will generate test images')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure camera: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the camera - start image capture."""
        try:
            self.get_logger().info('Activating camera...')
            
            # Activate publisher
            self.image_pub.on_activate()
            
            # Start capture timer
            self.timer = self.create_timer(0.033, self.capture_and_publish)  # ~30 FPS
            
            self.get_logger().info('Camera activated - capturing at 30 FPS')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to activate camera: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the camera - stop capture but keep hardware."""
        try:
            self.get_logger().info('Deactivating camera...')
            
            # Stop capture timer
            if self.timer:
                self.destroy_timer(self.timer)
                self.timer = None
            
            # Deactivate publisher
            self.image_pub.on_deactivate()
            
            self.get_logger().info('Camera deactivated - capture stopped')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate camera: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup the camera - release hardware resources."""
        try:
            self.get_logger().info('Cleaning up camera...')
            
            # Release camera hardware
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            
            # Destroy publisher
            if self.image_pub:
                self.destroy_lifecycle_publisher(self.image_pub)
                self.image_pub = None
            
            self.use_camera = False
            
            self.get_logger().info('Camera cleaned up successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to cleanup camera: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the camera."""
        self.get_logger().info('Camera shutting down')
        return TransitionCallbackReturn.SUCCESS

    def init_camera(self):
        """Initialize camera capture."""
        try:
            # Try to open camera device 0
            self.cap = cv2.VideoCapture(0)
            
            if not self.cap.isOpened():
                self.get_logger().warn('Could not open camera device 0')
                return False
            
            # Set camera properties for better performance
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Test capture
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Could not capture test frame')
                self.cap.release()
                return False
            
            self.get_logger().info(f'Camera initialized: {frame.shape[1]}x{frame.shape[0]} @ 30fps')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {e}')
            if self.cap:
                self.cap.release()
            return False

    def capture_and_publish(self):
        """Capture image and publish - only when active."""
        if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return
        
        try:
            if self.use_camera and self.cap is not None:
                # Capture from real camera
                ret, frame = self.cap.read()
                
                if not ret:
                    self.get_logger().warn('Failed to capture frame')
                    return
                
            else:
                # Generate test pattern when no camera
                frame = self.generate_test_image()
            
            # Convert to ROS Image message
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            
            # Publish image
            self.image_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error in capture_and_publish: {e}')

    def generate_test_image(self):
        """Generate a test image when no camera is available."""
        # Create a simple test pattern
        height, width = 480, 640
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Add some visual elements
        cv2.circle(frame, (width//2, height//2), 50, (0, 255, 0), -1)
        cv2.putText(frame, 'TEST IMAGE', (width//2 - 100, height//2 + 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Add timestamp for visual feedback
        import time
        timestamp = str(int(time.time() % 100))
        cv2.putText(frame, timestamp, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        return frame

def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_node = CameraLifecycleNode()
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            camera_node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
