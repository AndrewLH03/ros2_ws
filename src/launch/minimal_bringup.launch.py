#!/usr/bin/env python3
"""
Minimal Bringup Launch File for CR3 Control System

Launches minimal versions of the core nodes for rqt_graph visualization.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    """
    Outline: Generate a launch description for minimal system bringup.
    - Launches core nodes with dummy implementations
    - Enables visualization in rqt_graph
    """
    # Define the base path to our workspace
    workspace_dir = "/home/andrewlh/VSCode/ros2_ws"
    
    return LaunchDescription([
        # Camera Interface
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/camera_interface/bin/camera_node')],
            name='camera_node',
            output='screen'
        ),
        
        # Perception
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/perception/bin/hand_pose_node')],
            name='hand_pose_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/perception/bin/body_pose_node')],
            name='body_pose_node',
            output='screen'
        ),
        
        # Control
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/control/bin/pose_to_command_node')],
            name='pose_to_command_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/control/bin/motion_planner_node')],
            name='motion_planner_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/control/bin/teleop_node')],
            name='teleop_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/control/bin/trajectory_executor_node')],
            name='trajectory_executor_node',
            output='screen'
        ),
        
        # CR3 Interface
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/cr3_interface/bin/cr3_controller_node')],
            name='cr3_controller_node',
            output='screen'
        ),
        
        # Simulation Interface
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/sim_interface/bin/simulator_node')],
            name='simulator_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/sim_interface/bin/sim_world_interface_node')],
            name='sim_world_interface_node',
            output='screen'
        ),
        
        # UI
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/ui/bin/mode_switcher_node')],
            name='mode_switcher_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=[os.path.join(workspace_dir, 'install/ui/bin/ui_dashboard_node')],
            name='ui_dashboard_node',
            output='screen'
        )
    ])
