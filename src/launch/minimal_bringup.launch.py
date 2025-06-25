#!/usr/bin/env python3
"""
Minimal Bringup Launch File for CR3 Control System

Launches minimal versions of the core nodes for rqt_graph visualization.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Outline: Generate a launch description for minimal system bringup.
    - Launches core nodes with dummy implementations
    - Enables visualization in rqt_graph
    """
    return LaunchDescription([
        # Camera Interface
        Node(
            package='camera_interface',
            executable='camera_node.py',
            name='camera_node',
            output='screen'
        ),
        
        # Perception
        Node(
            package='perception',
            executable='hand_pose_node.py',
            name='hand_pose_node',
            output='screen'
        ),
        Node(
            package='perception',
            executable='body_pose_node.py',
            name='body_pose_node',
            output='screen'
        ),
        
        # Control
        Node(
            package='control',
            executable='pose_to_command_node.py',
            name='pose_to_command_node',
            output='screen'
        ),
        
        # CR3 Interface
        Node(
            package='cr3_interface',
            executable='cr3_controller_node.py',
            name='cr3_controller_node',
            output='screen'
        ),
        
        # UI
        Node(
            package='ui',
            executable='mode_switcher_node.py',
            name='mode_switcher_node',
            output='screen'
        )
    ])
