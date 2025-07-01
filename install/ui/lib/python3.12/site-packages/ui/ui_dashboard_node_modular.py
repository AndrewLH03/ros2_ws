#!/usr/bin/env python3
"""
UI Dashboard Node for CR3 Control System - Modular Version

Enhanced modular dashboard with fullscreen scaling and complete functionality.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
import cv2
from cv_bridge import CvBridge

# Import modular components
from .ui_window_manager import UIWindowManager
from .ui_button_controller import UIButtonController
from .ui_visualization import UIVisualization


class UIDashboardNode(Node):
    """
    Modular UI dashboard with fullscreen capabilities and all original functionality.
    """
    def __init__(self):
        """Initialize the modular UI dashboard node."""
        super().__init__('ui_dashboard_node')
        
        # OpenCV and image processing
        self.cv_bridge = CvBridge()
        self.current_frame = None
        
        # Status data storage
        self.status_data = {
            'mode': 'pose_tracking',
            'emergency_stop': False,
            'hand_confidence': 0.0,
            'hand_poses': None,
            'body_poses': None,
            'selected_hand': 'Right'  # Default to right hand
        }
        
        # Initialize window manager first
        self.window_manager = UIWindowManager()
        dimensions = self.window_manager.get_dimensions()
        scaling_factors = self.window_manager.get_scaling_factors()
        
        # Initialize modular components
        self.button_controller = UIButtonController(self, dimensions)
        self.visualizer = UIVisualization(scaling_factors)
        
        # Set up ROS2 interfaces
        self._setup_subscribers()
        self._setup_publishers()
        
        # Set mouse callback
        self.window_manager.set_mouse_callback(self.mouse_callback)
        
        # Main update timer
        self.timer = self.create_timer(0.033, self.update_display)  # ~30 FPS
        
        self.get_logger().info('Modular UI dashboard started in fullscreen mode')
    
    def _setup_subscribers(self):
        """Set up ROS2 subscribers."""
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        
        self.hand_confidence_sub = self.create_subscription(
            Float32, '/perception/hand_confidence', self.hand_confidence_callback, 10)
        
        self.hand_pose_sub = self.create_subscription(
            PoseArray, '/perception/hand_pose', self.hand_pose_callback, 10)
        
        self.body_pose_sub = self.create_subscription(
            PoseArray, '/perception/body_pose', self.body_pose_callback, 10)
        
        self.mode_sub = self.create_subscription(
            String, '/mode', self.mode_callback, 10)
        
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)
    
    def _setup_publishers(self):
        """Set up ROS2 publishers."""
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.hand_selection_pub = self.create_publisher(String, '/ui/hand_selection', 10)
    
    # Callback methods
    def camera_callback(self, image_msg):
        """Process incoming camera image."""
        try:
            self.current_frame = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')
    
    def hand_confidence_callback(self, confidence_msg):
        """Update hand confidence."""
        self.status_data['hand_confidence'] = confidence_msg.data
    
    def hand_pose_callback(self, pose_array_msg):
        """Update hand pose data."""
        self.status_data['hand_poses'] = pose_array_msg
    
    def body_pose_callback(self, pose_array_msg):
        """Update body pose data."""
        self.status_data['body_poses'] = pose_array_msg
    
    def mode_callback(self, mode_msg):
        """Update current mode."""
        self.status_data['mode'] = mode_msg.data
    
    def emergency_callback(self, emergency_msg):
        """Update emergency stop status."""
        self.status_data['emergency_stop'] = emergency_msg.data
    
    def update_display(self):
        """Main display update loop."""
        try:
            # Create camera frame with overlays
            camera_frame = self.current_frame
            if camera_frame is not None:
                camera_frame = camera_frame.copy()
                self.visualizer.draw_pose_overlay(camera_frame, self.status_data)
            
            # Create UI panel
            ui_panel = self.button_controller.create_ui_panel(self.status_data)
            
            # Combine frames
            combined_frame = self.window_manager.create_combined_frame(camera_frame, ui_panel)
            
            # Display frame
            self.window_manager.display_frame(combined_frame)
            
            # Handle keyboard input
            key = self.window_manager.handle_key_input()
            if not self.button_controller.handle_key_input(key):
                return  # Shutdown requested
            
        except Exception as e:
            self.get_logger().error(f'Error in display update: {e}')
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks."""
        if event == cv2.EVENT_LBUTTONDOWN:
            dimensions = self.window_manager.get_dimensions()
            camera_width = dimensions['camera_width']
            
            # Check if click is in UI panel area
            if x > camera_width:
                self.button_controller.handle_click(x, y, camera_width)


def main(args=None):
    rclpy.init(args=args)
    node = UIDashboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.window_manager.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
