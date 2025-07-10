#!/usr/bin/env python3
"""
UI Visualization Module for CR3 Control System

Handles pose visualization, overlays, and camera feed processing.
"""

import cv2
import numpy as np


class UIVisualization:
    """
    Manages pose visualization, overlays, and camera feed processing.
    """
    def __init__(self, scaling_factors):
        """
        Initialize the visualization module.
        
        Args:
            scaling_factors: Scaling factors for responsive visualization
        """
        self.scaling = scaling_factors
        
        # Scaled visualization parameters
        self.landmark_base_radius = 4
        self.connection_base_thickness = 2
        self.text_base_size = 0.3
        
        # Calculate scaled parameters
        self.landmark_radius = max(1, int(self.landmark_base_radius * self.scaling['width_scale']))
        self.connection_thickness = max(1, int(self.connection_base_thickness * self.scaling['width_scale']))
        self.text_size = self.text_base_size * self.scaling['font_scale']
        self.text_thickness = self.scaling['thickness_scale']
    
    def draw_pose_overlay(self, frame, status_data, show_debug=True):
        """Draw pose information overlay on camera frame."""
        h, w = frame.shape[:2]
        
        # Draw body pose landmarks
        self.draw_body_pose(frame, w, h, status_data['body_poses'])
        
        # Draw hand pose landmarks
        self.draw_hand_pose(frame, w, h, status_data['hand_poses'], show_debug)
    
    def draw_body_pose(self, frame, width, height, body_poses):
        """Draw body pose landmarks and connections."""
        if body_poses is None or not body_poses.poses:
            return
            
        poses = body_poses.poses
        
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
        landmark_radius = max(4, int(8 * self.scaling['width_scale']))
        border_thickness = max(1, int(2 * self.scaling['width_scale']))
        
        for i, pose in enumerate(poses[:17]):  # Only first 17 landmarks
            if i in landmark_colors:
                x = int(pose.position.x * width)
                y = int(pose.position.y * height)
                cv2.circle(frame, (x, y), landmark_radius, landmark_colors[i], -1)
                cv2.circle(frame, (x, y), landmark_radius + 2, (255, 255, 255), border_thickness)  # White border
        
        # Draw body connections
        connections = [
            (11, 12),  # shoulders
            (11, 13),  # left shoulder to elbow
            (13, 15),  # left elbow to wrist
            (12, 14),  # right shoulder to elbow
            (14, 16),  # right elbow to wrist
        ]
        
        connection_thickness = max(2, int(3 * self.scaling['width_scale']))
        
        for start_idx, end_idx in connections:
            if start_idx < len(poses) and end_idx < len(poses):
                start_pose = poses[start_idx]
                end_pose = poses[end_idx]
                
                start_x = int(start_pose.position.x * width)
                start_y = int(start_pose.position.y * height)
                end_x = int(end_pose.position.x * width)
                end_y = int(end_pose.position.y * height)
                
                cv2.line(frame, (start_x, start_y), (end_x, end_y), (0, 255, 0), connection_thickness)
    
    def draw_hand_pose(self, frame, width, height, hand_poses, show_debug=True):
        """Draw hand pose landmarks and connections."""
        if hand_poses is None or not hand_poses.poses:
            return
            
        poses = hand_poses.poses
        
        # MediaPipe hand landmark connections
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
        
        # Scaled connection thickness
        connection_thickness = max(1, int(2 * self.scaling['width_scale']))
        
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
                    cv2.line(frame, (start_x, start_y), (end_x, end_y), (255, 255, 255), connection_thickness)
            
            # Draw landmarks as circles on top of lines
            for i, pose in enumerate(poses[:21]):  # All 21 hand landmarks
                x = int(pose.position.x * width)
                y = int(pose.position.y * height)
                
                # Different colors and sizes for different parts of hand
                if i == 0:  # Wrist
                    color = (0, 255, 0)  # Green
                    radius = max(3, int(6 * self.scaling['width_scale']))
                elif i in [4, 8, 12, 16, 20]:  # Fingertips
                    color = (0, 0, 255)  # Red
                    radius = max(2, int(5 * self.scaling['width_scale']))
                else:  # Joint landmarks
                    color = (255, 0, 255)  # Magenta
                    radius = max(2, int(4 * self.scaling['width_scale']))
                
                cv2.circle(frame, (x, y), radius, color, -1)
                
                # Add landmark numbers for debugging (optional)
                if show_debug:
                    text_offset_x = max(4, int(8 * self.scaling['width_scale']))
                    text_offset_y = max(4, int(8 * self.scaling['height_scale']))
                    cv2.putText(frame, str(i), (x + text_offset_x, y - text_offset_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, self.text_size, (255, 255, 255), self.text_thickness)
        else:
            # If we don't have all 21 landmarks, just draw what we have
            default_radius = max(3, int(6 * self.scaling['width_scale']))
            text_offset_x = max(4, int(8 * self.scaling['width_scale']))
            text_offset_y = max(4, int(8 * self.scaling['height_scale']))
            
            for i, pose in enumerate(poses):
                x = int(pose.position.x * width)
                y = int(pose.position.y * height)
                cv2.circle(frame, (x, y), default_radius, (255, 255, 0), -1)
                cv2.putText(frame, str(i), (x + text_offset_x, y - text_offset_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, self.text_size, (255, 255, 255), self.text_thickness)
    
    def draw_info_overlay(self, frame, status_data):
        """Draw informational overlay on the camera frame."""
        # Get frame dimensions
        h, w = frame.shape[:2]
        
        # Scaled text parameters
        font_size = 0.7 * self.scaling['font_scale']
        font_thickness = max(1, self.text_thickness)
        line_height = max(25, int(30 * self.scaling['height_scale']))
        margin = max(10, int(15 * self.scaling['width_scale']))
        
        # Draw semi-transparent background for text
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, line_height * 3), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
        
        # Current mode
        mode_text = f"Mode: {status_data['mode']}"
        cv2.putText(frame, mode_text, (margin, line_height), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_size, (255, 255, 255), font_thickness)
        
        # Hand tracking status
        hand_text = f"Tracking: {status_data['selected_hand']} Hand"
        cv2.putText(frame, hand_text, (margin, line_height * 2), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_size, (255, 255, 255), font_thickness)
        
        # Confidence
        confidence = status_data['hand_confidence']
        confidence_color = (0, 255, 0) if confidence > 0.5 else (0, 165, 255) if confidence > 0.3 else (0, 0, 255)
        confidence_text = f"Confidence: {confidence:.2f}"
        cv2.putText(frame, confidence_text, (margin, line_height * 3), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_size, confidence_color, font_thickness)
    
    def apply_mirror_effect(self, frame):
        """Apply horizontal mirror effect to the frame."""
        return cv2.flip(frame, 1)
    
    def add_crosshair(self, frame):
        """Add a crosshair to the center of the frame for reference."""
        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        line_length = max(20, int(30 * self.scaling['width_scale']))
        line_thickness = max(1, int(2 * self.scaling['width_scale']))
        
        # Draw crosshair
        cv2.line(frame, (center_x - line_length, center_y), (center_x + line_length, center_y), 
                (255, 255, 255), line_thickness)
        cv2.line(frame, (center_x, center_y - line_length), (center_x, center_y + line_length), 
                (255, 255, 255), line_thickness)
        
        # Draw center dot
        cv2.circle(frame, (center_x, center_y), max(2, int(3 * self.scaling['width_scale'])), 
                  (255, 255, 255), -1)
    
    def draw_fps_counter(self, frame, fps):
        """Draw FPS counter on the frame."""
        h, w = frame.shape[:2]
        
        font_size = 0.6 * self.scaling['font_scale']
        font_thickness = max(1, self.text_thickness)
        margin = max(10, int(15 * self.scaling['width_scale']))
        
        fps_text = f"FPS: {fps:.1f}"
        text_size = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, font_size, font_thickness)[0]
        
        # Position in top-right corner
        text_x = w - text_size[0] - margin
        text_y = margin + text_size[1]
        
        # Draw background
        cv2.rectangle(frame, (text_x - 5, text_y - text_size[1] - 5), 
                     (text_x + text_size[0] + 5, text_y + 5), (0, 0, 0), -1)
        
        # Draw text
        cv2.putText(frame, fps_text, (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, font_size, (255, 255, 255), font_thickness)
