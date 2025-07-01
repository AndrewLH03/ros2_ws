#!/usr/bin/env python3
"""
UI Components Module for ROS 2 Integration

Adapted from hand tracking dashboard UI components to work with ROS 2 nodes.
Provides OpenCV-based UI elements for the CR3 control system.
"""

import cv2
import numpy as np
from typing import Dict, Any, Tuple, Optional


def draw_button(img, x, y, w, h, text, bg_color, text_color=(255, 255, 255)):
    """Draw a labeled rectangle button on the given image."""
    cv2.rectangle(img, (x, y), (x + w, y + h), bg_color, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), (200, 200, 200), 2)  # Border
    
    # Center text in button
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
    text_x = x + (w - text_size[0]) // 2
    text_y = y + (h + text_size[1]) // 2
    cv2.putText(img, text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)


def draw_coordinates_panel(ui_frame, frame_w, shoulder_pose=None, wrist_pose=None):
    """Draw coordinate information panel for pose data."""
    y_offset = 40
    line_height = 25
    
    if shoulder_pose and wrist_pose:
        # Shoulder coordinates
        cv2.putText(ui_frame, "Shoulder (Robot Frame):", 
                   (frame_w + 10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        y_offset += line_height
        cv2.putText(ui_frame, f"X: {shoulder_pose.position.x:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(ui_frame, f"Y: {shoulder_pose.position.y:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(ui_frame, f"Z: {shoulder_pose.position.z:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += line_height
        
        # Wrist coordinates
        cv2.putText(ui_frame, "Wrist (Robot Frame):", 
                   (frame_w + 10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        y_offset += line_height
        cv2.putText(ui_frame, f"X: {wrist_pose.position.x:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(ui_frame, f"Y: {wrist_pose.position.y:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(ui_frame, f"Z: {wrist_pose.position.z:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)


def draw_vector_panel(ui_frame, frame_w, tracking_vector=None):
    """Draw tracking vector information panel."""
    y_offset = 200
    line_height = 25
    
    if tracking_vector:
        cv2.putText(ui_frame, "Tracking Vector:", 
                   (frame_w + 10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        y_offset += line_height
        
        # Calculate magnitude
        magnitude = (tracking_vector.x**2 + tracking_vector.y**2 + tracking_vector.z**2)**0.5
        
        cv2.putText(ui_frame, f"X: {tracking_vector.x:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(ui_frame, f"Y: {tracking_vector.y:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(ui_frame, f"Z: {tracking_vector.z:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(ui_frame, f"Mag: {magnitude:.3f}", 
                   (frame_w + 20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)


def draw_system_status(ui_frame, frame_w, status_data):
    """Draw system status indicators."""
    y_offset = 300
    line_height = 25
    
    # Mode
    mode = status_data.get('mode', 'unknown')
    mode_color = (0, 255, 0) if mode == 'pose_tracking' else (255, 255, 0)
    cv2.putText(ui_frame, f"Mode: {mode.upper()}", 
               (frame_w + 10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)
    y_offset += line_height
    
    # Hand confidence
    confidence = status_data.get('hand_confidence', 0.0)
    conf_color = (0, 255, 0) if confidence > 0.7 else (255, 165, 0) if confidence > 0.4 else (0, 0, 255)
    cv2.putText(ui_frame, f"Hand Conf: {confidence:.2f}", 
               (frame_w + 10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, conf_color, 1)
    y_offset += line_height
    
    # Emergency stop status
    emergency = status_data.get('emergency_stop', False)
    emerg_color = (0, 0, 255) if emergency else (0, 255, 0)
    emerg_text = "EMERGENCY" if emergency else "NORMAL"
    cv2.putText(ui_frame, f"Status: {emerg_text}", 
               (frame_w + 10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, emerg_color, 2)


def create_control_panel(ui_frame, frame_w, frame_h, ui_state):
    """Create control buttons panel."""
    button_y = frame_h - 200
    button_h = 35
    button_w = 160
    button_spacing = 45
    button_x = frame_w + 20
    
    # Pause/Resume button
    pause_text = "RESUME" if ui_state.get('paused', False) else "PAUSE"
    pause_color = (0, 255, 0) if ui_state.get('paused', False) else (255, 165, 0)
    draw_button(ui_frame, button_x, button_y, button_w, button_h, 
                pause_text, pause_color)
    
    # Emergency stop button
    button_y += button_spacing
    draw_button(ui_frame, button_x, button_y, button_w, button_h, 
                "EMERGENCY", (0, 0, 255))
    
    # Mode cycle button
    button_y += button_spacing
    draw_button(ui_frame, button_x, button_y, button_w, button_h, 
                "CYCLE MODE", (100, 100, 255))
    
    # Debug toggle
    button_y += button_spacing
    debug_text = "HIDE DEBUG" if ui_state.get('show_debug', True) else "SHOW DEBUG"
    draw_button(ui_frame, button_x, button_y, button_w, button_h, 
                debug_text, (128, 128, 128))


def create_comprehensive_ui(frame, status_data, ui_state):
    """Create a comprehensive UI overlay combining all elements."""
    if frame is None:
        return None
        
    frame_h, frame_w = frame.shape[:2]
    panel_width = 320
    
    # Create extended frame with UI panel
    ui_frame = np.zeros((frame_h, frame_w + panel_width, 3), dtype=np.uint8)
    
    # Copy original frame
    ui_frame[:, :frame_w, :] = frame
    
    # Dark panel background
    ui_frame[:, frame_w:, :] = (40, 40, 40)
    
    # Add panel border
    cv2.line(ui_frame, (frame_w, 0), (frame_w, frame_h), (100, 100, 100), 2)
    
    # Draw all UI elements
    draw_coordinates_panel(ui_frame, frame_w, 
                          status_data.get('shoulder_pose'), 
                          status_data.get('wrist_pose'))
    
    draw_vector_panel(ui_frame, frame_w, status_data.get('tracking_vector'))
    
    draw_system_status(ui_frame, frame_w, status_data)
    
    create_control_panel(ui_frame, frame_w, frame_h, ui_state)
    
    return ui_frame


def get_button_regions(frame_w, frame_h):
    """Return clickable regions for UI buttons."""
    button_y = frame_h - 200
    button_h = 35
    button_w = 160
    button_spacing = 45
    button_x = frame_w + 20
    
    regions = {
        'pause': (button_x, button_y, button_w, button_h),
        'emergency': (button_x, button_y + button_spacing, button_w, button_h),
        'mode_cycle': (button_x, button_y + 2*button_spacing, button_w, button_h),
        'debug_toggle': (button_x, button_y + 3*button_spacing, button_w, button_h)
    }
    
    return regions


def check_button_click(x, y, regions):
    """Check which button (if any) was clicked."""
    for button_name, (bx, by, bw, bh) in regions.items():
        if bx <= x <= bx + bw and by <= y <= by + bh:
            return button_name
    return None


def draw_pose_landmarks_on_frame(frame, shoulder_pose, wrist_pose):
    """Draw pose landmarks directly on the camera frame."""
    if frame is None:
        return frame
        
    h, w = frame.shape[:2]
    
    # Draw shoulder
    if shoulder_pose:
        # Convert normalized coordinates to pixel coordinates
        shoulder_x = int(shoulder_pose.position.x * w)
        shoulder_y = int(shoulder_pose.position.y * h)
        cv2.circle(frame, (shoulder_x, shoulder_y), 8, (0, 0, 255), -1)
        cv2.putText(frame, "SHOULDER", (shoulder_x+15, shoulder_y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Draw wrist
    if wrist_pose:
        wrist_x = int(wrist_pose.position.x * w)
        wrist_y = int(wrist_pose.position.y * h)
        cv2.circle(frame, (wrist_x, wrist_y), 8, (255, 0, 0), -1)
        cv2.putText(frame, "WRIST", (wrist_x+15, wrist_y-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Draw tracking vector line
        if shoulder_pose:
            shoulder_x = int(shoulder_pose.position.x * w)
            shoulder_y = int(shoulder_pose.position.y * h)
            cv2.line(frame, (shoulder_x, shoulder_y), (wrist_x, wrist_y), (255, 0, 255), 3)
    
    return frame
