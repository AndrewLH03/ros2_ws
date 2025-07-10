#!/usr/bin/env python3
"""
Mode Manager Node

Enhanced mode switcher that manages the lifecycle of nodes based on control mode.
Starts/stops camera interface and other nodes as needed.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import signal
import os
import time


class ModeManagerNode(Node):
    """
    Mode manager that handles mode switching and node lifecycle.
    
    Features:
    - Publishes /mode topic
    - Starts/stops camera interface based on mode
    - Manages perception nodes lifecycle
    - Handles mode transitions cleanly
    """
    
    def __init__(self):
        super().__init__('mode_manager_node')
        
        # Available modes
        self.modes = ["manual", "perception"]
        self.current_mode = "manual"  # Default to manual
        
        # Track running processes
        self.running_processes = {}
        
        # Publishers
        self.mode_pub = self.create_publisher(String, '/mode', 10)
        
        # Subscribers
        self.ui_command_sub = self.create_subscription(
            String, '/ui/command', self.handle_ui_commands, 10)
            
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.handle_emergency_stop, 10)
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.check_process_health)
        
        self.get_logger().info('Mode manager node started')
        
        # Publish initial mode
        self.publish_mode(self.current_mode)
    
    def handle_ui_commands(self, msg):
        """Handle commands from UI."""
        command = msg.data
        if command.startswith('set_mode:'):
            mode = command.split(':')[1]
            self.set_mode(mode)
        elif command == 'cycle_mode':
            self.cycle_mode()
    
    def handle_emergency_stop(self, msg):
        """Handle emergency stop - switch to manual mode."""
        if msg.data:
            self.get_logger().warn('🛑 Emergency stop - switching to manual mode')
            self.set_mode('manual')
    
    def cycle_mode(self):
        """Cycle through available modes."""
        current_index = self.modes.index(self.current_mode) if self.current_mode in self.modes else 0
        next_index = (current_index + 1) % len(self.modes)
        new_mode = self.modes[next_index]
        self.set_mode(new_mode)
    
    def set_mode(self, mode):
        """Set the control mode and manage node lifecycle."""
        if mode not in self.modes:
            self.get_logger().warn(f'Invalid mode: {mode}')
            return
        
        if mode == self.current_mode:
            self.get_logger().info(f'Already in {mode} mode')
            return
        
        old_mode = self.current_mode
        self.get_logger().info(f'Mode transition: {old_mode} -> {mode}')
        
        # Stop nodes for old mode
        self.stop_mode_nodes(old_mode)
        
        # Start nodes for new mode
        self.start_mode_nodes(mode)
        
        # Update current mode
        self.current_mode = mode
        
        # Publish mode change
        self.publish_mode(mode)
        
        self.get_logger().info(f'✅ Mode transition complete: {mode}')
    
    def stop_mode_nodes(self, mode):
        """Stop nodes associated with a specific mode."""
        if mode == "perception":
            self.get_logger().info('🛑 Stopping perception nodes...')
            
            # Stop camera and hand pose nodes
            self.stop_process('camera_node')
            self.stop_process('hand_pose_node')
            
            # Kill any remaining perception processes
            try:
                subprocess.run(['pkill', '-f', 'camera_node'], check=False)
                subprocess.run(['pkill', '-f', 'hand_pose_node'], check=False)
                time.sleep(1)  # Give time for cleanup
            except Exception as e:
                self.get_logger().warn(f'Error killing perception processes: {e}')
    
    def start_mode_nodes(self, mode):
        """Start nodes associated with a specific mode."""
        if mode == "perception":
            self.get_logger().info('🚀 Starting perception nodes...')
            
            try:
                # Start camera node
                camera_process = subprocess.Popen([
                    'ros2', 'run', 'camera_interface', 'camera_node'
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
                self.running_processes['camera_node'] = camera_process
                self.get_logger().info('Camera node started')
                
                # Wait for camera to initialize
                time.sleep(2)
                
                # Start hand pose node
                hand_pose_process = subprocess.Popen([
                    'ros2', 'run', 'perception', 'hand_pose_node'
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
                self.running_processes['hand_pose_node'] = hand_pose_process
                self.get_logger().info('Hand pose node started')
                
            except Exception as e:
                self.get_logger().error(f'Failed to start perception nodes: {e}')
                # Fallback to manual mode if perception fails
                self.current_mode = "manual"
                self.publish_mode("manual")
        
        elif mode == "manual":
            self.get_logger().info('🎮 Manual mode - no additional nodes needed')
    
    def stop_process(self, process_name):
        """Stop a specific process."""
        if process_name in self.running_processes:
            process = self.running_processes[process_name]
            try:
                process.terminate()
                process.wait(timeout=3)
                self.get_logger().info(f'{process_name} stopped gracefully')
            except subprocess.TimeoutExpired:
                process.kill()
                self.get_logger().warn(f'{process_name} force killed')
            except Exception as e:
                self.get_logger().error(f'Error stopping {process_name}: {e}')
            finally:
                del self.running_processes[process_name]
    
    def check_process_health(self):
        """Check health of running processes."""
        dead_processes = []
        
        for name, process in self.running_processes.items():
            if process.poll() is not None:
                self.get_logger().warn(f'Process {name} has died')
                dead_processes.append(name)
        
        # Remove dead processes
        for name in dead_processes:
            del self.running_processes[name]
        
        # If perception mode and camera died, switch to manual
        if (self.current_mode == "perception" and 
            'camera_node' in dead_processes):
            self.get_logger().error('Camera node died - switching to manual mode')
            self.set_mode('manual')
    
    def publish_mode(self, mode):
        """Publish the current mode."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Mode published: {mode}')
    
    def shutdown_node(self):
        """Clean shutdown of all managed processes."""
        self.get_logger().info('Shutting down mode manager...')
        
        # Stop all running processes
        for name in list(self.running_processes.keys()):
            self.stop_process(name)
        
        self.get_logger().info('Mode manager shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mode manager stopped by user')
    finally:
        node.shutdown_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
