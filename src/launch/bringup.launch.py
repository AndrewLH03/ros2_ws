#!/usr/bin/env python3
"""
Complete Hand Tracking System Bringup Launch File

Launches all components of the enhanced CR3 hand tracking system.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate complete launch description for hand tracking system."""
    
    # Get package directories
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Declare launch arguments
    use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to use real camera or simulation'
    )
    
    enable_dashboard = DeclareLaunchArgument(
        'enable_dashboard',
        default_value='true',
        description='Enable OpenCV dashboard interface'
    )
    
    default_mode = DeclareLaunchArgument(
        'default_mode',
        default_value='pose_tracking',
        description='Default control mode'
    )
    
    camera_device = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='Camera device ID'
    )
    
    # Include perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'perception.launch.py')
        ),
        launch_arguments={
            'use_camera': LaunchConfiguration('use_camera'),
            'camera_device': LaunchConfiguration('camera_device')
        }.items()
    )
    
    # Include control launch file
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'control.launch.py')
        ),
        launch_arguments={
            'default_mode': LaunchConfiguration('default_mode')
        }.items()
    )
    
    # Include UI dashboard launch file
    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ui_dashboard.launch.py')
        ),
        launch_arguments={
            'enable_dashboard': LaunchConfiguration('enable_dashboard')
        }.items()
    )
    
    # Include diagnostics (if needed)
    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'diagnostics.launch.py')
        )
    )
    
    return LaunchDescription([
        # Launch arguments
        use_camera,
        enable_dashboard,
        default_mode,
        camera_device,
        
        # Core system components
        GroupAction([
            perception_launch,
            control_launch,
            ui_launch
        ]),
        
        # Optional diagnostics
        # diagnostics_launch,  # Uncomment if diagnostics are implemented
    ])
