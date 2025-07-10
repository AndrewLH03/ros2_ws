#!/usr/bin/env python3
"""
UI Window Manager for CR3 Control System - Fixed Version

Manages the main OpenCV window, fullscreen mode, and display scaling.
"""

import cv2
import numpy as np


class UIWindowManager:
    """
    Manages the main UI window with fullscreen capabilities and proper scaling.
    """
    def __init__(self, window_name="CR3 Hand Tracking Dashboard"):
        """Initialize the window manager."""
        self.window_name = window_name
        self.is_fullscreen = True
        
        # Create window and set to fullscreen immediately
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        # Detect screen dimensions
        self.screen_width = 1920
        self.screen_height = 1080
        self._detect_screen_size()
        
        # UI layout parameters (properly scaled)
        self.panel_width = max(400, int(self.screen_width * 0.25))  # 25% of screen width, min 400px
        self.camera_width = self.screen_width - self.panel_width
        
        print(f"Window Manager initialized: {self.screen_width}x{self.screen_height}")
        print(f"Camera area: {self.camera_width}x{self.screen_height}")
        print(f"Panel area: {self.panel_width}x{self.screen_height}")
        
    def _detect_screen_size(self):
        """Detect screen size for proper scaling."""
        try:
            import tkinter as tk
            root = tk.Tk()
            self.screen_width = root.winfo_screenwidth()
            self.screen_height = root.winfo_screenheight()
            root.destroy()
            print(f"Screen size detected: {self.screen_width}x{self.screen_height}")
        except Exception as e:
            print(f"Could not detect screen size: {e}, using defaults")
            # Fallback to common screen sizes
            self.screen_width = 1920
            self.screen_height = 1080
        
        # Recalculate camera width
        self.panel_width = max(400, int(self.screen_width * 0.25))
        self.camera_width = self.screen_width - self.panel_width
    
    def create_combined_frame(self, camera_frame, ui_panel):
        """Create a combined frame with camera and UI panel properly scaled."""
        if camera_frame is None:
            camera_frame = self._create_default_frame()
        
        # Scale camera frame to fit available space while maintaining aspect ratio
        camera_frame_scaled = self._scale_camera_frame(camera_frame)
        
        # Create the final combined frame at full screen resolution
        combined_frame = np.zeros((self.screen_height, self.screen_width, 3), dtype=np.uint8)
        
        # Place scaled camera frame (centered in camera area)
        h, w = camera_frame_scaled.shape[:2]
        y_offset = max(0, (self.screen_height - h) // 2)
        x_offset = max(0, (self.camera_width - w) // 2)
        
        # Ensure we don't go out of bounds
        end_y = min(y_offset + h, self.screen_height)
        end_x = min(x_offset + w, self.camera_width)
        actual_h = end_y - y_offset
        actual_w = end_x - x_offset
        
        combined_frame[y_offset:end_y, x_offset:end_x, :] = camera_frame_scaled[:actual_h, :actual_w, :]
        
        # Place UI panel (ensure it fits in the panel area)
        if ui_panel is not None:
            panel_h, panel_w = ui_panel.shape[:2]
            # Scale panel if needed
            if panel_h > self.screen_height or panel_w > self.panel_width:
                scale_h = self.screen_height / panel_h
                scale_w = self.panel_width / panel_w
                scale = min(scale_h, scale_w)
                new_h = int(panel_h * scale)
                new_w = int(panel_w * scale)
                ui_panel = cv2.resize(ui_panel, (new_w, new_h))
                panel_h, panel_w = new_h, new_w
            
            # Place panel in the right area
            end_panel_h = min(panel_h, self.screen_height)
            end_panel_w = min(panel_w, self.panel_width)
            combined_frame[:end_panel_h, self.camera_width:self.camera_width+end_panel_w, :] = ui_panel[:end_panel_h, :end_panel_w, :]
        
        return combined_frame
    
    def _scale_camera_frame(self, frame):
        """Scale camera frame to fit available space while maintaining aspect ratio."""
        h, w = frame.shape[:2]
        
        # Calculate scale to fit in camera area
        scale_w = self.camera_width / w
        scale_h = self.screen_height / h
        scale = min(scale_w, scale_h)  # Use smaller scale to maintain aspect ratio
        
        new_w = int(w * scale)
        new_h = int(h * scale)
        
        return cv2.resize(frame, (new_w, new_h))
    
    def _create_default_frame(self):
        """Create a default frame when no camera feed is available."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add waiting message
        text1 = 'Waiting for camera feed...'
        text2 = 'Check camera connection'
        
        cv2.putText(frame, text1, (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, text2, (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 1)
        
        return frame
    
    def display_frame(self, frame):
        """Display the frame in the window."""
        cv2.imshow(self.window_name, frame)
    
    def handle_key_input(self):
        """Handle key input and return the key pressed."""
        return cv2.waitKey(1) & 0xFF
    
    def set_mouse_callback(self, callback):
        """Set mouse callback for the window."""
        cv2.setMouseCallback(self.window_name, callback)
    
    def toggle_fullscreen(self):
        """Toggle between fullscreen and windowed mode."""
        self.is_fullscreen = not self.is_fullscreen
        if self.is_fullscreen:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)
    
    def cleanup(self):
        """Clean up resources."""
        cv2.destroyWindow(self.window_name)
    
    def get_dimensions(self):
        """Get current dimensions for other components."""
        return {
            'screen_width': self.screen_width,
            'screen_height': self.screen_height,
            'camera_width': self.camera_width,
            'panel_width': self.panel_width
        }
    
    def get_scaling_factors(self):
        """Get scaling factors for UI elements."""
        # Base scaling factors (assuming 1920x1080 as reference)
        width_scale = self.screen_width / 1920.0
        height_scale = self.screen_height / 1080.0
        
        return {
            'width_scale': width_scale,
            'height_scale': height_scale,
            'font_scale': min(width_scale, height_scale),  # Use smaller scale for fonts
            'thickness_scale': max(1, int(min(width_scale, height_scale)))
        }
