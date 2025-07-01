#!/usr/bin/env python3
"""
UI Dashboard Node for CR3 Control System

Enhanced with hand tracking dashboard features. Provides OpenCV-based visualization,
system monitoring, and control interface.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Vector3
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import threading

class UIDashboardNode(Node):
    """
    Enhanced UI dashboard with hand tracking visualization.
    - Subscribes to diagnostic/status topics and camera feed
    - Displays OpenCV-based dashboard interface
    - Provides real-time monitoring and control
    - Integrates hand tracking visualization features
    """
    def __init__(self):
        """Initialize the UI dashboard node."""
        super().__init__('ui_dashboard_node')
        
        # OpenCV and image processing
        self.cv_bridge = CvBridge()
        self.current_frame = None
        self.display_frame = None
        
        # Store status information
        self.status_data = {
            'mode': 'pose_tracking',
            'emergency_stop': False,
            'hand_confidence': 0.0,
            'hand_poses': None,  # PoseArray for hand landmarks
            'body_poses': None,  # PoseArray for body landmarks
            'shoulder_pose': None,
            'wrist_pose': None,
            'tracking_vector': None,
            'robot_target': None
        }
        
        # UI state
        self.ui_state = {
            'running': True,
            'paused': False,
            'show_debug': True,
            'mirror_camera': False
        }
        
        # Subscribe to topics
        self.subscribe_to_topics()
        
        # Publishers
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.ui_commands_pub = self.create_publisher(String, '/ui/control_commands', 10)
        
        # Timer for UI updates
        self.timer = self.create_timer(0.033, self.update_display)  # ~30 FPS
        
        # OpenCV window setup
        self.window_name = 'CR3 Hand Tracking Dashboard'
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        self.get_logger().info('Enhanced UI dashboard started with OpenCV visualization')

    def subscribe_to_topics(self):
        """Subscribe to all relevant topics."""
        # Camera feed
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        
        # Perception topics
        self.hand_confidence_sub = self.create_subscription(
            Float32, '/perception/hand_confidence', self.hand_confidence_callback, 10)
        
        self.hand_pose_sub = self.create_subscription(
            PoseArray, '/perception/hand_pose', self.hand_pose_callback, 10)
        
        self.body_pose_sub = self.create_subscription(
            PoseArray, '/perception/body_pose', self.body_pose_callback, 10)
        
        self.shoulder_pose_sub = self.create_subscription(
            Pose, '/perception/shoulder_pose', self.shoulder_pose_callback, 10)
        
        self.wrist_pose_sub = self.create_subscription(
            Pose, '/perception/wrist_pose', self.wrist_pose_callback, 10)
        
        self.tracking_vector_sub = self.create_subscription(
            Vector3, '/perception/tracking_vector', self.tracking_vector_callback, 10)
        
        # Control topics
        self.mode_sub = self.create_subscription(
            String, '/mode', self.mode_callback, 10)
        
        self.target_pose_sub = self.create_subscription(
            Pose, '/cr3/target_pose', self.target_pose_callback, 10)
        
        # System topics
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)

    def handle_robot_status(self, msg):
        """Handle robot status messages."""
        self.status_data['robot_status'] = msg.data
        self.display_status('Robot status: ' + msg.data)

    def handle_joint_states(self, msg):
        """Handle joint state messages."""
        # Store only latest joint positions
        if len(msg.position) > 0:
            self.status_data['joint_positions'] = msg.position
            # Only log at debug level to avoid flooding
            self.get_logger().debug(f'Received joint positions: {msg.position}')

    def handle_target_pose(self, msg):
        """Handle target pose messages."""
        self.status_data['target_pose'] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        self.display_status(f'Target pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}')

    def handle_health(self, msg):
        """Handle health status messages."""
        self.status_data['health'] = msg.data
        self.display_status('Health: ' + msg.data)

    def handle_log(self, msg):
        """Handle log messages."""
        self.display_status('Log: ' + msg.data)

    def display_status(self, status):
        """Display system status in the UI."""
        # For minimal implementation, just log to console
        self.get_logger().info(status)

    def publish_mode_change(self, mode):
        """Publish a mode change command."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.display_status(f'Changed mode to: {mode}')

    def publish_emergency_stop(self, stop=True):
        """Publish an emergency stop command."""
        msg = Bool()
        msg.data = stop
        self.emergency_pub.publish(msg)
        self.display_status(f'Emergency stop: {stop}')

    def publish_demo_commands(self):
        """Publish demo commands for visualization."""
        # Cycle through modes
        next_mode = self.modes[self.current_mode_index]
        self.publish_mode_change(next_mode)
        
        self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)

    # Callback methods for data collection
    def camera_callback(self, msg):
        """Process camera feed for visualization."""
        try:
            self.current_frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().debug(f'Received camera frame: {self.current_frame.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting camera image: {e}')

    def hand_confidence_callback(self, msg):
        """Store hand detection confidence."""
        self.status_data['hand_confidence'] = msg.data

    def hand_pose_callback(self, msg):
        """Store hand pose landmarks."""
        self.status_data['hand_poses'] = msg

    def body_pose_callback(self, msg):
        """Store body pose landmarks."""
        self.status_data['body_poses'] = msg

    def shoulder_pose_callback(self, msg):
        """Store shoulder pose."""
        self.status_data['shoulder_pose'] = msg

    def wrist_pose_callback(self, msg):
        """Store wrist pose."""
        self.status_data['wrist_pose'] = msg

    def tracking_vector_callback(self, msg):
        """Store tracking vector."""
        self.status_data['tracking_vector'] = msg

    def mode_callback(self, msg):
        """Store current mode."""
        self.status_data['mode'] = msg.data

    def target_pose_callback(self, msg):
        """Store robot target pose."""
        self.status_data['robot_target'] = msg

    def emergency_callback(self, msg):
        """Store emergency stop status."""
        self.status_data['emergency_stop'] = msg.data

    def update_display(self):
        """Update the OpenCV display with current data."""
        # Create a default frame if no camera data available
        if self.current_frame is None:
            # Create a default "waiting for camera" frame
            default_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(default_frame, 'Waiting for camera feed...', (200, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(default_frame, 'Check camera connection', (210, 280), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 1)
            display_frame = default_frame
        else:
            # Create a copy for drawing
            display_frame = self.current_frame.copy()
            
            # Draw overlay information
            self.draw_pose_overlay(display_frame)
            self.draw_tracking_vector(display_frame)
        
        # Create UI panel and combine
        ui_frame = self.create_ui_panel(display_frame)
        
        # Display the frame
        cv2.imshow(self.window_name, ui_frame)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        self.handle_key_input(key)

    def draw_pose_overlay(self, frame):
        """Draw pose information overlay on camera frame."""
        h, w = frame.shape[:2]
        
        # Draw body pose landmarks
        self.draw_body_pose(frame, w, h)
        
        # Draw hand pose landmarks
        self.draw_hand_pose(frame, w, h)

    def draw_body_pose(self, frame, width, height):
        """Draw body pose landmarks and connections."""
        if self.status_data['body_poses'] is None or not self.status_data['body_poses'].poses:
            return
            
        poses = self.status_data['body_poses'].poses
        
        # MediaPipe pose landmark order (first 17 landmarks are the main ones)
        # 0: nose, 11: left_shoulder, 12: right_shoulder, 13: left_elbow, 14: right_elbow,
        # 15: left_wrist, 16: right_wrist, etc.
        
        # Draw landmarks as circles
        landmark_colors = {
            0: (0, 255, 255),    # nose - yellow
            11: (0, 255, 0),     # left_shoulder - green
            12: (0, 255, 0),     # right_shoulder - green
            13: (255, 0, 0),     # left_elbow - red
            14: (255, 0, 0),     # right_elbow - red
            15: (255, 255, 0),   # left_wrist - cyan
            16: (255, 255, 0),   # right_wrist - cyan
        }
        
        # Draw key landmarks
        for i, pose in enumerate(poses[:17]):  # Only first 17 landmarks
            if i in landmark_colors:
                x = int(pose.position.x * width)
                y = int(pose.position.y * height)
                cv2.circle(frame, (x, y), 8, landmark_colors[i], -1)
                cv2.circle(frame, (x, y), 10, (255, 255, 255), 2)  # White border
        
        # Draw body connections
        connections = [
            (11, 12),  # shoulders
            (11, 13),  # left shoulder to elbow
            (13, 15),  # left elbow to wrist
            (12, 14),  # right shoulder to elbow
            (14, 16),  # right elbow to wrist
        ]
        
        for start_idx, end_idx in connections:
            if start_idx < len(poses) and end_idx < len(poses):
                start_pose = poses[start_idx]
                end_pose = poses[end_idx]
                
                start_x = int(start_pose.position.x * width)
                start_y = int(start_pose.position.y * height)
                end_x = int(end_pose.position.x * width)
                end_y = int(end_pose.position.y * height)
                
                cv2.line(frame, (start_x, start_y), (end_x, end_y), (0, 255, 0), 3)

    def draw_hand_pose(self, frame, width, height):
        """Draw hand pose landmarks and connections just like MediaPipe reference."""
        if self.status_data['hand_poses'] is None or not self.status_data['hand_poses'].poses:
            return
            
        poses = self.status_data['hand_poses'].poses
        
        # MediaPipe hand landmark connections (same as reference script)
        # Each hand has exactly 21 landmarks (0-20)
        hand_connections = [
            # Thumb
            (0, 1), (1, 2), (2, 3), (3, 4),
            # Index finger  
            (0, 5), (5, 6), (6, 7), (7, 8),
            # Middle finger
            (0, 9), (9, 10), (10, 11), (11, 12),
            # Ring finger
            (0, 13), (13, 14), (14, 15), (15, 16),
            # Pinky
            (0, 17), (17, 18), (18, 19), (19, 20)
        ]
        
        # Ensure we have exactly 21 landmarks (MediaPipe hand model)
        if len(poses) >= 21:
            # Draw connections first (lines between landmarks)
            for connection in hand_connections:
                if connection[0] < len(poses) and connection[1] < len(poses):
                    start_pose = poses[connection[0]]
                    end_pose = poses[connection[1]]
                    
                    start_x = int(start_pose.position.x * width)
                    start_y = int(start_pose.position.y * height)
                    end_x = int(end_pose.position.x * width)
                    end_y = int(end_pose.position.y * height)
                    
                    # Draw connection line
                    cv2.line(frame, (start_x, start_y), (end_x, end_y), (255, 255, 255), 2)
            
            # Draw landmarks as circles on top of lines
            for i, pose in enumerate(poses[:21]):  # All 21 hand landmarks
                x = int(pose.position.x * width)
                y = int(pose.position.y * height)
                
                # Different colors for different parts of hand
                if i == 0:  # Wrist
                    color = (0, 255, 0)  # Green
                    radius = 6
                elif i in [4, 8, 12, 16, 20]:  # Fingertips
                    color = (0, 0, 255)  # Red
                    radius = 5
                else:  # Joint landmarks
                    color = (255, 0, 255)  # Magenta
                    radius = 4
                
                cv2.circle(frame, (x, y), radius, color, -1)
                
                # Add landmark numbers for debugging (optional)
                if self.ui_state.get('show_debug', True):
                    cv2.putText(frame, str(i), (x+8, y-8), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
        else:
            # If we don't have all 21 landmarks, just draw what we have
            for i, pose in enumerate(poses):
                x = int(pose.position.x * width)
                y = int(pose.position.y * height)
                cv2.circle(frame, (x, y), 6, (255, 255, 0), -1)
                cv2.putText(frame, str(i), (x+8, y-8), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

    def draw_tracking_vector(self, frame):
        """Draw tracking vector visualization."""
        if (self.status_data['shoulder_pose'] is not None and 
            self.status_data['wrist_pose'] is not None):
            
            h, w = frame.shape[:2]
            shoulder_x = int(self.status_data['shoulder_pose'].position.x * w)
            shoulder_y = int(self.status_data['shoulder_pose'].position.y * h)
            wrist_x = int(self.status_data['wrist_pose'].position.x * w)
            wrist_y = int(self.status_data['wrist_pose'].position.y * h)
            
            # Draw tracking vector line
            cv2.line(frame, (shoulder_x, shoulder_y), (wrist_x, wrist_y), (255, 0, 255), 3)

    def create_ui_panel(self, frame):
        """Create UI control panel and combine with camera frame."""
        h, w = frame.shape[:2]
        panel_width = 350
        
        # Create extended frame with UI panel
        ui_frame = np.zeros((h, w + panel_width, 3), dtype=np.uint8)
        ui_frame[:, :w, :] = frame
        ui_frame[:, w:, :] = (40, 40, 40)  # Dark gray background
        
        # Draw UI elements
        self.draw_status_panel(ui_frame, w)
        self.draw_control_buttons(ui_frame, w)
        
        return ui_frame

    def draw_status_panel(self, frame, offset_x):
        """Draw status information panel."""
        y_pos = 30
        line_height = 30
        
        # Mode
        cv2.putText(frame, f"Mode: {self.status_data['mode']}", 
                   (offset_x + 10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y_pos += line_height
        
        # Hand confidence
        confidence = self.status_data['hand_confidence']
        color = (0, 255, 0) if confidence > 0.5 else (0, 165, 255) if confidence > 0.3 else (0, 0, 255)
        cv2.putText(frame, f"Hand Confidence: {confidence:.2f}", 
                   (offset_x + 10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        y_pos += line_height
        
        # Hand landmarks count
        hand_count = len(self.status_data['hand_poses'].poses) if self.status_data['hand_poses'] else 0
        cv2.putText(frame, f"Hand Landmarks: {hand_count}", 
                   (offset_x + 10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_pos += line_height
        
        # Body landmarks count
        body_count = len(self.status_data['body_poses'].poses) if self.status_data['body_poses'] else 0
        cv2.putText(frame, f"Body Landmarks: {body_count}", 
                   (offset_x + 10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_pos += line_height
        
        # Emergency status
        emerg_color = (0, 0, 255) if self.status_data['emergency_stop'] else (0, 255, 0)
        emerg_text = "EMERGENCY STOP" if self.status_data['emergency_stop'] else "NORMAL"
        cv2.putText(frame, f"Status: {emerg_text}", 
                   (offset_x + 10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, emerg_color, 2)
        y_pos += line_height * 2
        
        # Coordinate information
        if self.status_data['tracking_vector'] is not None:
            vec = self.status_data['tracking_vector']
            cv2.putText(frame, "Tracking Vector:", 
                       (offset_x + 10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_pos += line_height
            cv2.putText(frame, f"X: {vec.x:.3f}", 
                       (offset_x + 20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y_pos += 20
            cv2.putText(frame, f"Y: {vec.y:.3f}", 
                       (offset_x + 20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y_pos += 20
            cv2.putText(frame, f"Z: {vec.z:.3f}", 
                       (offset_x + 20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    def draw_control_buttons(self, frame, offset_x):
        """Draw control buttons."""
        button_y = 300
        button_h = 40
        button_w = 180
        button_spacing = 50
        
        # Pause/Resume button
        pause_text = "RESUME" if self.ui_state['paused'] else "PAUSE"
        pause_color = (0, 255, 0) if self.ui_state['paused'] else (255, 165, 0)
        self.draw_button(frame, offset_x + 10, button_y, button_w, button_h, 
                        pause_text, pause_color)
        
        # Emergency stop button
        button_y += button_spacing
        self.draw_button(frame, offset_x + 10, button_y, button_w, button_h, 
                        "EMERGENCY STOP", (0, 0, 255))
        
        # Mode cycle button
        button_y += button_spacing
        self.draw_button(frame, offset_x + 10, button_y, button_w, button_h, 
                        "CYCLE MODE", (100, 100, 255))

    def draw_button(self, frame, x, y, w, h, text, bg_color, text_color=(255, 255, 255)):
        """Draw a button on the frame."""
        cv2.rectangle(frame, (x, y), (x + w, y + h), bg_color, -1)
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        text_x = x + (w - text_size[0]) // 2
        text_y = y + (h + text_size[1]) // 2
        cv2.putText(frame, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks on UI elements."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Check which button was clicked
            frame_w = self.current_frame.shape[1] if self.current_frame is not None else 640
            if x > frame_w:  # Clicked in UI panel
                self.handle_ui_click(x - frame_w, y)

    def handle_ui_click(self, x, y):
        """Handle clicks in the UI panel."""
        button_w = 180
        button_h = 40
        
        # Check pause button (y=300)
        if 10 <= x <= 10 + button_w and 300 <= y <= 300 + button_h:
            self.ui_state['paused'] = not self.ui_state['paused']
            self.get_logger().info(f"UI {'paused' if self.ui_state['paused'] else 'resumed'}")
            
        # Check emergency button (y=350)
        elif 10 <= x <= 10 + button_w and 350 <= y <= 350 + button_h:
            self.publish_emergency_stop()
            
        # Check mode cycle button (y=400)
        elif 10 <= x <= 10 + button_w and 400 <= y <= 400 + button_h:
            self.cycle_mode()

    def handle_key_input(self, key):
        """Handle keyboard input."""
        if key == ord('q'):
            self.ui_state['running'] = False
            rclpy.shutdown()
        elif key == ord('p'):
            self.ui_state['paused'] = not self.ui_state['paused']
        elif key == ord('m'):
            self.cycle_mode()
        elif key == ord('e'):
            self.publish_emergency_stop()

    def cycle_mode(self):
        """Cycle through available modes."""
        modes = ["manual", "pose_tracking", "vector_control", "autonomous"]
        current_idx = modes.index(self.status_data['mode']) if self.status_data['mode'] in modes else 0
        next_idx = (current_idx + 1) % len(modes)
        
        mode_msg = String()
        mode_msg.data = modes[next_idx]
        self.mode_pub.publish(mode_msg)
        self.get_logger().info(f"Mode changed to: {modes[next_idx]}")

    def publish_emergency_stop(self):
        """Publish emergency stop command."""
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_pub.publish(stop_msg)
        self.get_logger().warn("Emergency stop activated")



def main(args=None):
    rclpy.init(args=args)
    node = UIDashboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()