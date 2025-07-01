#!/usr/bin/env python3
"""
UI Button Controller for CR3 Control System

Handles all button interactions, layout, and command publishing.
"""

import cv2
import numpy as np
from std_msgs.msg import String, Bool
import subprocess


class UIButtonController:
    """
    Manages all UI buttons, interactions, and command publishing.
    """
    def __init__(self, node, dimensions):
        """Initialize the button controller."""
        self.node = node
        self.dimensions = dimensions
        
        # Button layout parameters (scaled to screen size)
        self.panel_width = dimensions['panel_width']
        self.screen_height = dimensions['screen_height']
        
        # Scale button sizes based on screen height
        scale_factor = self.screen_height / 1080  # Base scale on 1080p
        self.button_height = int(50 * scale_factor)
        self.button_width = int(300 * scale_factor)
        self.button_spacing = int(60 * scale_factor)
        self.margin = int(20 * scale_factor)
        self.font_scale = 0.8 * scale_factor
        self.font_thickness = max(1, int(2 * scale_factor))
        
        # Button positions (will be calculated)
        self.buttons = {}
        self._calculate_button_positions()
        
        # Button states
        self.button_states = {
            'paused': False,
            'emergency': False,
            'selected_hand': 'Right'
        }
    
    def _calculate_button_positions(self):
        """Calculate button positions based on screen size."""
        y_start = int(200 * (self.screen_height / 1080))  # Start below status panel
        
        self.buttons = {
            'pause': {'x': self.margin, 'y': y_start, 'w': self.button_width, 'h': self.button_height},
            'emergency': {'x': self.margin, 'y': y_start + self.button_spacing, 'w': self.button_width, 'h': self.button_height},
            'cycle_mode': {'x': self.margin, 'y': y_start + 2 * self.button_spacing, 'w': self.button_width, 'h': self.button_height},
            'left_hand': {'x': self.margin, 'y': y_start + 3 * self.button_spacing, 'w': self.button_width // 2 - 5, 'h': self.button_height},
            'right_hand': {'x': self.margin + self.button_width // 2 + 5, 'y': y_start + 3 * self.button_spacing, 'w': self.button_width // 2 - 5, 'h': self.button_height},
            'stop_all': {'x': self.margin, 'y': y_start + 4 * self.button_spacing, 'w': self.button_width, 'h': self.button_height}
        }
    
    def create_ui_panel(self, status_data):
        """Create the UI panel with status and buttons."""
        panel = np.zeros((self.screen_height, self.panel_width, 3), dtype=np.uint8)
        panel[:, :, :] = (40, 40, 40)  # Dark gray background
        
        # Draw status information
        self._draw_status_info(panel, status_data)
        
        # Draw buttons
        self._draw_buttons(panel, status_data)
        
        return panel
    
    def _draw_status_info(self, panel, status_data):
        """Draw status information at the top of the panel."""
        y_pos = int(40 * (self.screen_height / 1080))
        line_height = int(35 * (self.screen_height / 1080))
        
        # Scale font for status text
        status_font_scale = 0.6 * (self.screen_height / 1080)
        status_thickness = max(1, int(2 * (self.screen_height / 1080)))
        
        # Mode
        cv2.putText(panel, f"Mode: {status_data.get('mode', 'Unknown')}", 
                   (self.margin, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                   status_font_scale, (255, 255, 255), status_thickness)
        y_pos += line_height
        
        # Hand confidence
        confidence = status_data.get('hand_confidence', 0.0)
        color = (0, 255, 0) if confidence > 0.5 else (0, 165, 255) if confidence > 0.3 else (0, 0, 255)
        cv2.putText(panel, f"Hand Confidence: {confidence:.2f}", 
                   (self.margin, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                   status_font_scale, color, status_thickness)
        y_pos += line_height
        
        # Selected hand
        cv2.putText(panel, f"Tracking: {status_data.get('selected_hand', 'Right')} Hand", 
                   (self.margin, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                   status_font_scale, (255, 255, 255), status_thickness)
        y_pos += line_height
        
        # Hand landmarks count
        hand_count = len(status_data.get('hand_poses', {}).get('poses', [])) if status_data.get('hand_poses') else 0
        cv2.putText(panel, f"Hand Landmarks: {hand_count}", 
                   (self.margin, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                   status_font_scale, (255, 255, 255), status_thickness)
        y_pos += line_height
        
        # Body landmarks count  
        body_count = len(status_data.get('body_poses', {}).get('poses', [])) if status_data.get('body_poses') else 0
        cv2.putText(panel, f"Body Landmarks: {body_count}", 
                   (self.margin, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                   status_font_scale, (255, 255, 255), status_thickness)
        y_pos += line_height
        
        # Emergency status
        emerg_color = (0, 0, 255) if status_data.get('emergency_stop', False) else (0, 255, 0)
        emerg_text = "EMERGENCY STOP" if status_data.get('emergency_stop', False) else "NORMAL"
        cv2.putText(panel, f"Status: {emerg_text}", 
                   (self.margin, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                   status_font_scale, emerg_color, status_thickness)
    
    def _draw_buttons(self, panel, status_data):
        """Draw all control buttons."""
        # Pause/Resume button
        pause_text = "RESUME" if self.button_states['paused'] else "PAUSE"
        pause_color = (0, 255, 0) if self.button_states['paused'] else (255, 165, 0)
        self._draw_button(panel, 'pause', pause_text, pause_color)
        
        # Emergency stop button
        self._draw_button(panel, 'emergency', "EMERGENCY STOP", (0, 0, 255))
        
        # Mode cycle button
        self._draw_button(panel, 'cycle_mode', "CYCLE MODE", (100, 100, 255))
        
        # Hand selection buttons
        left_color = (0, 255, 0) if status_data.get('selected_hand') == 'Left' else (128, 128, 128)
        right_color = (0, 255, 0) if status_data.get('selected_hand') == 'Right' else (128, 128, 128)
        
        self._draw_button(panel, 'left_hand', "LEFT", left_color)
        self._draw_button(panel, 'right_hand', "RIGHT", right_color)
        
        # Stop all nodes button
        self._draw_button(panel, 'stop_all', "STOP ALL NODES", (255, 0, 0))
    
    def _draw_button(self, panel, button_key, text, bg_color, text_color=(255, 255, 255)):
        """Draw a single button."""
        btn = self.buttons[button_key]
        x, y, w, h = btn['x'], btn['y'], btn['w'], btn['h']
        
        # Draw button background
        cv2.rectangle(panel, (x, y), (x + w, y + h), bg_color, -1)
        cv2.rectangle(panel, (x, y), (x + w, y + h), (255, 255, 255), 2)  # Border
        
        # Draw button text
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, self.font_thickness)[0]
        text_x = x + (w - text_size[0]) // 2
        text_y = y + (h + text_size[1]) // 2
        cv2.putText(panel, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 
                   self.font_scale, text_color, self.font_thickness)
    
    def handle_click(self, x, y, camera_width):
        """Handle mouse clicks on buttons."""
        # Adjust coordinates to panel space
        panel_x = x - camera_width
        panel_y = y
        
        # Check each button
        for button_name, btn in self.buttons.items():
            if (btn['x'] <= panel_x <= btn['x'] + btn['w'] and 
                btn['y'] <= panel_y <= btn['y'] + btn['h']):
                self._handle_button_press(button_name)
                return True
        
        return False
    
    def _handle_button_press(self, button_name):
        """Handle button press actions."""
        if button_name == 'pause':
            self.button_states['paused'] = not self.button_states['paused']
            self.node.get_logger().info(f"UI {'paused' if self.button_states['paused'] else 'resumed'}")
            
        elif button_name == 'emergency':
            self._toggle_emergency_stop()
            
        elif button_name == 'cycle_mode':
            self._cycle_mode()
            
        elif button_name == 'left_hand':
            self._select_hand('Left')
            
        elif button_name == 'right_hand':
            self._select_hand('Right')
            
        elif button_name == 'stop_all':
            self._stop_all_nodes()
    
    def _toggle_emergency_stop(self):
        """Toggle emergency stop."""
        self.button_states['emergency'] = not self.button_states['emergency']
        
        # Publish emergency stop command
        msg = Bool()
        msg.data = self.button_states['emergency']
        self.node.emergency_pub.publish(msg)
        
        status = "activated" if self.button_states['emergency'] else "deactivated"
        self.node.get_logger().info(f'Emergency stop {status}')
    
    def _cycle_mode(self):
        """Cycle through available modes."""
        modes = ["manual", "pose_tracking", "vector_control", "autonomous"]
        current_mode = self.node.status_data.get('mode', 'pose_tracking')
        
        try:
            current_index = modes.index(current_mode)
            next_index = (current_index + 1) % len(modes)
            new_mode = modes[next_index]
            
            # Publish mode change
            msg = String()
            msg.data = new_mode
            self.node.mode_pub.publish(msg)
            self.node.get_logger().info(f'Mode cycled to: {new_mode}')
            
        except ValueError:
            self.node.get_logger().warn(f'Unknown current mode: {current_mode}')
    
    def _select_hand(self, hand):
        """Select which hand to track."""
        if hand in ['Left', 'Right']:
            self.button_states['selected_hand'] = hand
            self.node.status_data['selected_hand'] = hand
            
            # Publish hand selection command
            msg = String()
            msg.data = hand
            self.node.hand_selection_pub.publish(msg)
            self.node.get_logger().info(f'Hand selection changed to: {hand}')
    
    def _stop_all_nodes(self):
        """Stop all ROS nodes and shutdown the system."""
        self.node.get_logger().info('Stopping all ROS nodes...')
        
        try:
            # Kill all ROS2 processes
            subprocess.run(['pkill', '-f', 'ros2'], check=False)
            subprocess.run(['pkill', '-f', 'hand_pose_node'], check=False)
            subprocess.run(['pkill', '-f', 'body_pose_node'], check=False)
            subprocess.run(['pkill', '-f', 'camera_node'], check=False)
            subprocess.run(['pkill', '-f', 'mode_switcher_node'], check=False)
            subprocess.run(['pkill', '-f', 'pose_to_command_node'], check=False)
            subprocess.run(['pkill', '-f', 'coordinate_transform_node'], check=False)
            
            self.node.get_logger().info('All nodes stopped successfully')
            
        except Exception as e:
            self.node.get_logger().error(f'Error stopping nodes: {e}')
        
        # Signal UI to shut down
        import rclpy
        rclpy.shutdown()
    
    def handle_key_input(self, key):
        """Handle keyboard shortcuts."""
        if key == ord('p'):  # 'p' for pause/resume
            self._handle_button_press('pause')
        elif key == ord('e'):  # 'e' for emergency stop
            self._handle_button_press('emergency')
        elif key == ord('m'):  # 'm' for mode cycle
            self._handle_button_press('cycle_mode')
        elif key == ord('l'):  # 'l' for left hand
            self._handle_button_press('left_hand')
        elif key == ord('r'):  # 'r' for right hand
            self._handle_button_press('right_hand')
        elif key == ord('s'):  # 's' for stop all nodes
            self._handle_button_press('stop_all')
        elif key == ord('q') or key == 27:  # 'q' or ESC for quit
            import rclpy
            rclpy.shutdown()
            return False  # Signal to stop the main loop
        
        return True  # Continue running
