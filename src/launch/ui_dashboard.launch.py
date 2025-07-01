#!/usr/bin/env python3
"""
UI Dashboard Launch File for CR3 Hand Tracking System

Launches UI-related nodes including enhanced dashboard and mode switcher.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for UI nodes."""
    
    # Declare launch arguments
    enable_dashboard = DeclareLaunchArgument(
        'enable_dashboard',
        default_value='true',
        description='Enable OpenCV dashboard interface'
    )
    
    mirror_camera = DeclareLaunchArgument(
        'mirror_camera',
        default_value='false',
        description='Mirror camera feed for user-facing setup'
    )
    
    debug_mode = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug visualization'
    )
    
    # Enhanced UI dashboard node
    ui_dashboard_node = Node(
        package='ui',
        executable='ui_dashboard_node',
        name='ui_dashboard_node',
        parameters=[{
            'enable_dashboard': LaunchConfiguration('enable_dashboard'),
            'mirror_camera': LaunchConfiguration('mirror_camera'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            'update_rate': 30.0,
            'window_name': 'CR3 Hand Tracking Dashboard'
        }],
        condition=LaunchConfiguration('enable_dashboard')
    )
    
    # Enhanced mode switcher node
    mode_switcher_node = Node(
        package='ui',
        executable='mode_switcher_node',
        name='mode_switcher_node',
        parameters=[{
            'control_mode': 'pose_tracking',
            'debounce_time': 1.0,
            'available_modes': ['manual', 'pose_tracking', 'vector_control', 'autonomous']
        }]
    )
    
    return LaunchDescription([
        enable_dashboard,
        mirror_camera,
        debug_mode,
        ui_dashboard_node,
        mode_switcher_node
    ])
