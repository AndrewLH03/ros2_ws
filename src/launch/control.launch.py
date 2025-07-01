#!/usr/bin/env python3
"""
Control Launch File for CR3 Hand Tracking System

Launches all control-related nodes including enhanced pose-to-command mapping.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for control nodes."""
    
    # Declare launch arguments
    default_mode = DeclareLaunchArgument(
        'default_mode',
        default_value='pose_tracking',
        description='Default control mode (manual, pose_tracking, vector_control, autonomous)'
    )
    
    safety_enabled = DeclareLaunchArgument(
        'safety_enabled',
        default_value='true',
        description='Enable safety constraints'
    )
    
    # Enhanced pose to command node
    pose_to_command_node = Node(
        package='control',
        executable='pose_to_command_node',
        name='pose_to_command_node',
        parameters=[{
            'default_mode': LaunchConfiguration('default_mode'),
            'safety_enabled': LaunchConfiguration('safety_enabled'),
            'scaling_factor': 0.5,
            'max_velocity': 1.0,
            'workspace_limits': {
                'x_min': -0.5, 'x_max': 0.5,
                'y_min': -0.5, 'y_max': 0.5,
                'z_min': 0.1, 'z_max': 0.8
            }
        }]
    )
    
    # Motion planner node
    motion_planner_node = Node(
        package='control',
        executable='motion_planner_node',
        name='motion_planner_node',
        parameters=[{
            'planning_time': 5.0,
            'smoothing_enabled': True
        }]
    )
    
    # Trajectory executor node
    trajectory_executor_node = Node(
        package='control',
        executable='trajectory_executor_node',
        name='trajectory_executor_node',
        parameters=[{
            'execution_timeout': 10.0,
            'interpolation_rate': 100.0
        }]
    )
    
    # Teleop node (backup control)
    teleop_node = Node(
        package='control',
        executable='teleop_node',
        name='teleop_node',
        parameters=[{
            'linear_scale': 0.1,
            'angular_scale': 0.1
        }]
    )
    
    return LaunchDescription([
        default_mode,
        safety_enabled,
        pose_to_command_node,
        motion_planner_node,
        trajectory_executor_node,
        teleop_node
    ])
