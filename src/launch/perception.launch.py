"""
Perception Launch File for CR3 Control System

Outline-only version. Starts only the nodes related to video input and pose recognition.
Includes: camera_node or sim_camera_node, camera_info_node, hand_pose_node, body_pose_node, pose_filter_node.
Useful for debugging MediaPipe pose recognition in isolation. Configurable launch argument for real/sim camera.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Outline: Generate a launch description for perception subsystem.
    - Launches camera, pose, and filter nodes
    - Supports real or simulated camera
    """
    pass