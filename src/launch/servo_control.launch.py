#!/usr/bin/env python3
"""
Servo Control Launch File

Launches servo control nodes for Dynamixel servos.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for servo control."""
    
    # Declare launch arguments
    enable_hardware = DeclareLaunchArgument(
        'enable_hardware',
        default_value='false',
        description='Enable actual hardware interface (requires Dynamixel SDK)'
    )
    
    servo_device = DeclareLaunchArgument(
        'servo_device',
        default_value='/dev/ttyUSB0',
        description='Servo communication device'
    )
    
    servo_baudrate = DeclareLaunchArgument(
        'servo_baudrate',
        default_value='57600',
        description='Servo communication baudrate'
    )
    
    # Finger servo controller node
    finger_controller_node = Node(
        package='servo_control',
        executable='finger_servo_controller_node',
        name='finger_servo_controller',
        output='screen',
        parameters=[{
            'smoothing_factor': 0.8,
            'max_position_change': 200,
            'control_frequency': 20.0
        }]
    )
    
    # Hardware interface node (conditional on enable_hardware)
    servo_interface_node = Node(
        package='servo_control',
        executable='servo_interface_node',
        name='servo_interface',
        output='screen',
        parameters=[{
            'device_name': LaunchConfiguration('servo_device'),
            'baudrate': LaunchConfiguration('servo_baudrate'),
            'enable_hardware': LaunchConfiguration('enable_hardware')
        }],
        condition=None  # Always launch, but node will detect hardware availability
    )
    
    return LaunchDescription([
        # Launch arguments
        enable_hardware,
        servo_device,
        servo_baudrate,
        
        # Servo control nodes
        finger_controller_node,
        servo_interface_node,
    ])
