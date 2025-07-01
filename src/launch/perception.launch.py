#!/usr/bin/env python3
"""
Perception Launch File for CR3 Hand Tracking System

Launches all perception-related nodes including enhanced coordinate transformation.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for perception nodes."""
    
    # Declare launch arguments
    use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to launch camera node'
    )
    
    camera_device = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='Camera device ID'
    )
    
    # Camera interface node
    camera_node = Node(
        package='camera_interface',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'device_id': LaunchConfiguration('camera_device'),
            'width': 640,
            'height': 480,
            'fps': 30
        }],
        condition=LaunchConfiguration('use_camera')
    )
    
    # Hand pose detection node
    hand_pose_node = Node(
        package='perception',
        executable='hand_pose_node',
        name='hand_pose_node',
        parameters=[{
            'confidence_threshold': 0.5,
            'tracking_confidence': 0.5
        }]
    )
    
    # Body pose detection node
    body_pose_node = Node(
        package='perception',
        executable='body_pose_node',
        name='body_pose_node',
        parameters=[{
            'confidence_threshold': 0.5,
            'tracking_confidence': 0.5
        }]
    )
    
    # Enhanced coordinate transformation node
    coordinate_transform_node = Node(
        package='perception',
        executable='coordinate_transform_node',
        name='coordinate_transform_node',
        parameters=[{
            'camera_frame': 'camera_link',
            'robot_frame': 'robot_base',
            'scaling_factor': 0.5
        }]
    )
    
    # Pose filter node (optional)
    pose_filter_node = Node(
        package='perception',
        executable='pose_filter_node',
        name='pose_filter_node',
        parameters=[{
            'filter_strength': 0.8
        }]
    )
    
    return LaunchDescription([
        use_camera,
        camera_device,
        camera_node,
        hand_pose_node,
        body_pose_node,
        coordinate_transform_node,
        pose_filter_node
    ])
