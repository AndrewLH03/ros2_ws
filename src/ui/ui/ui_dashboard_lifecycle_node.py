#!/usr/bin/env python3
"""
UI Dashboard Lifecycle Node for CR3 Control System

Lifecycle version of the UI dashboard with proper state management.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState
from std_msgs.msg import String, Bool, Float32, Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
import cv2
from cv_bridge import CvBridge

# Import modular components
from .ui_window_manager import UIWindowManager
from .ui_button_controller import UIButtonController
from .ui_visualization import UIVisualization


class UIDashboardLifecycleNode(LifecycleNode):
    """
    Lifecycle UI dashboard with fullscreen capabilities and proper state management.
    """
    def __init__(self):
        """Initialize the lifecycle UI dashboard node."""
        super().__init__('ui_dashboard_node')
        
        # Initialize core variables that don't depend on active state
        self.cv_bridge = None
        self.current_frame = None
        self.status_data = {}
        self.window_manager = None
        self.button_controller = None
        self.visualizer = None
        self.timer = None
        
        # Subscribers and publishers
        self.camera_sub = None
        self.hand_confidence_sub = None
        self.hand_pose_sub = None
        self.body_pose_sub = None
        self.mode_sub = None
        self.emergency_sub = None
        self.finger_curl_sub = None
        self.manual_servo_status_sub = None
        
        self.mode_pub = None
        self.emergency_pub = None
        self.hand_selection_pub = None
        
        self.get_logger().info('UI Dashboard Lifecycle Node initialized')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the node - set up resources but don't start processing."""
        self.get_logger().info('Configuring UI Dashboard Node')
        
        try:
            # Initialize OpenCV and image processing
            self.cv_bridge = CvBridge()
            self.current_frame = None
            
            # Initialize status data storage
            self.status_data = {
                'mode': 'manual',
                'emergency_stop': False,
                'hand_confidence': 0.0,
                'hand_poses': None,
                'body_poses': None,
                'selected_hand': 'Right',
                'finger_curl_ratios': None,
                'manual_servo_status': None
            }
            
            # Initialize window manager
            self.window_manager = UIWindowManager()
            dimensions = self.window_manager.get_dimensions()
            scaling_factors = self.window_manager.get_scaling_factors()
            
            # Initialize modular components
            self.button_controller = UIButtonController(self, dimensions)
            self.visualizer = UIVisualization(scaling_factors)
            
            # Set mouse callback
            self.window_manager.set_mouse_callback(self.mouse_callback)
            
            # Set up ROS2 interfaces
            self._setup_subscribers()
            self._setup_publishers()
            
            self.get_logger().info('UI Dashboard Node configured successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to configure UI Dashboard Node: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the node - start processing and timers."""
        self.get_logger().info('Activating UI Dashboard Node')
        
        try:
            # Activate publishers
            if self.mode_pub:
                self.mode_pub.on_activate(state)
            if self.emergency_pub:
                self.emergency_pub.on_activate(state)
            if self.hand_selection_pub:
                self.hand_selection_pub.on_activate(state)
            
            # Start the main update timer
            self.timer = self.create_timer(0.033, self.update_display)  # 30 FPS
            
            self.get_logger().info('UI Dashboard Node activated successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to activate UI Dashboard Node: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the node - stop processing but keep resources."""
        self.get_logger().info('Deactivating UI Dashboard Node')
        
        try:
            # Stop the timer
            if self.timer:
                self.timer.cancel()
                self.timer = None
            
            # Deactivate publishers
            if self.mode_pub:
                self.mode_pub.on_deactivate(state)
            if self.emergency_pub:
                self.emergency_pub.on_deactivate(state)
            if self.hand_selection_pub:
                self.hand_selection_pub.on_deactivate(state)
            
            self.get_logger().info('UI Dashboard Node deactivated successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to deactivate UI Dashboard Node: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup the node - release all resources."""
        self.get_logger().info('Cleaning up UI Dashboard Node')
        
        try:
            # Destroy subscribers
            if self.camera_sub:
                self.destroy_subscription(self.camera_sub)
            if self.hand_confidence_sub:
                self.destroy_subscription(self.hand_confidence_sub)
            if self.hand_pose_sub:
                self.destroy_subscription(self.hand_pose_sub)
            if self.body_pose_sub:
                self.destroy_subscription(self.body_pose_sub)
            if self.mode_sub:
                self.destroy_subscription(self.mode_sub)
            if self.emergency_sub:
                self.destroy_subscription(self.emergency_sub)
            if self.finger_curl_sub:
                self.destroy_subscription(self.finger_curl_sub)
            if self.manual_servo_status_sub:
                self.destroy_subscription(self.manual_servo_status_sub)
            
            # Destroy publishers
            if self.mode_pub:
                self.destroy_publisher(self.mode_pub)
            if self.emergency_pub:
                self.destroy_publisher(self.emergency_pub)
            if self.hand_selection_pub:
                self.destroy_publisher(self.hand_selection_pub)
            
            # Clean up OpenCV windows
            if self.window_manager:
                cv2.destroyAllWindows()
            
            # Reset all variables
            self.cv_bridge = None
            self.current_frame = None
            self.status_data = {}
            self.window_manager = None
            self.button_controller = None
            self.visualizer = None
            
            self.get_logger().info('UI Dashboard Node cleaned up successfully')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Failed to cleanup UI Dashboard Node: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the node."""
        self.get_logger().info('Shutting down UI Dashboard Node')
        
        # Clean up any remaining resources
        cv2.destroyAllWindows()
        
        return TransitionCallbackReturn.SUCCESS

    def _setup_subscribers(self):
        """Set up ROS2 subscribers."""
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 1)
        
        self.hand_confidence_sub = self.create_subscription(
            Float32, '/perception/hand_confidence', self.hand_confidence_callback, 1)
        
        self.hand_pose_sub = self.create_subscription(
            PoseArray, '/perception/hand_pose', self.hand_pose_callback, 1)
        
        self.body_pose_sub = self.create_subscription(
            PoseArray, '/perception/body_pose', self.body_pose_callback, 1)
        
        self.mode_sub = self.create_subscription(
            String, '/mode', self.mode_callback, 10)
        
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)
            
        self.finger_curl_sub = self.create_subscription(
            Float32MultiArray, '/perception/finger_curl_ratios', self.finger_curl_callback, 10)
            
        self.manual_servo_status_sub = self.create_subscription(
            String, '/manual/servo_status', self.manual_servo_status_callback, 10)

    def _setup_publishers(self):
        """Set up ROS2 publishers."""
        self.mode_pub = self.create_lifecycle_publisher(String, '/mode', 10)
        self.emergency_pub = self.create_lifecycle_publisher(Bool, '/emergency_stop', 10)
        self.hand_selection_pub = self.create_lifecycle_publisher(String, '/ui/hand_selection', 10)

    # Callback methods - same as original implementation
    def camera_callback(self, msg):
        """Handle camera image updates."""
        if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return
            
        try:
            self.current_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')

    def hand_confidence_callback(self, msg):
        """Handle hand confidence updates."""
        if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.status_data['hand_confidence'] = msg.data

    def hand_pose_callback(self, msg):
        """Handle hand pose updates."""
        if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.status_data['hand_poses'] = msg

    def body_pose_callback(self, msg):
        """Handle body pose updates."""
        if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.status_data['body_poses'] = msg

    def mode_callback(self, msg):
        """Handle mode change updates."""
        if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.status_data['mode'] = msg.data

    def emergency_callback(self, msg):
        """Handle emergency stop updates."""
        if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.status_data['emergency_stop'] = msg.data

    def finger_curl_callback(self, msg):
        """Handle finger curl ratio updates."""
        if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.status_data['finger_curl_ratios'] = msg.data

    def manual_servo_status_callback(self, msg):
        """Handle manual servo status updates."""
        if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.status_data['manual_servo_status'] = msg.data

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events."""
        if (self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE and 
            self.button_controller):
            self.button_controller.handle_mouse_event(event, x, y, flags, param)

    def update_display(self):
        """Main display update loop."""
        if self.get_current_state().id != LifecycleState.PRIMARY_STATE_ACTIVE:
            return
            
        if not all([self.window_manager, self.button_controller, self.visualizer]):
            return
            
        try:
            # Create the display frame
            display_frame = self.window_manager.create_display_frame()
            
            # Add camera feed if available
            if self.current_frame is not None:
                camera_area = self.window_manager.get_camera_area()
                resized_frame = cv2.resize(self.current_frame, 
                                         (camera_area['width'], camera_area['height']))
                display_frame[camera_area['y']:camera_area['y']+camera_area['height'],
                            camera_area['x']:camera_area['x']+camera_area['width']] = resized_frame
            
            # Add UI elements
            display_frame = self.button_controller.draw_buttons(display_frame)
            display_frame = self.visualizer.draw_status_info(display_frame, self.status_data)
            
            # Show the frame
            self.window_manager.show_frame(display_frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key != 255:  # A key was pressed
                self.button_controller.handle_keyboard_input(key)
                
        except Exception as e:
            self.get_logger().error(f'Display update error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        lifecycle_node = UIDashboardLifecycleNode()
        
        # Keep the node running
        rclpy.spin(lifecycle_node)
        
    except Exception as e:
        print(f'Error starting UI Dashboard Lifecycle Node: {e}')
    finally:
        try:
            lifecycle_node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
