#!/usr/bin/env python3
"""
Perception Control Node

Handles camera-based perception control by coordinating with camera and hand pose nodes.
Part of the unified control system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import signal
import os


class PerceptionControlNode(Node):
    """
    Perception control coordinator.
    
    Manages camera and hand pose nodes based on control mode.
    Starts/stops camera interface when switching to/from perception mode.
    """
    
    def __init__(self):
        super().__init__('perception_control_node')
        
        # Control state
        self.current_mode = "manual"
        self.perception_active = False
        self.camera_process = None
        self.hand_pose_process = None
        
        # Subscribers
        self.mode_sub = self.create_subscription(
            String, '/mode', self.handle_mode_change, 10)
            
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.handle_emergency_stop, 10)
        
        # Publishers
        self.perception_status_pub = self.create_publisher(
            String, '/perception/status', 10)
        
        # Status timer
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info('Perception control node started')
    
    def handle_mode_change(self, msg):
        """Handle control mode changes."""
        new_mode = msg.data
        if new_mode != self.current_mode:
            self.get_logger().info(f'Mode change: {self.current_mode} -> {new_mode}')
            old_mode = self.current_mode
            self.current_mode = new_mode
            
            if new_mode == "perception" and old_mode != "perception":
                self.start_perception_nodes()
            elif old_mode == "perception" and new_mode != "perception":
                self.stop_perception_nodes()
    
    def handle_emergency_stop(self, msg):
        """Handle emergency stop - stop all perception if active."""
        if msg.data and self.perception_active:
            self.get_logger().warn('🛑 Emergency stop - stopping perception nodes')
            self.stop_perception_nodes()
    
    def start_perception_nodes(self):
        """Start camera and hand pose nodes for perception mode."""
        self.get_logger().info('🚀 Starting perception nodes...')
        
        try:
            # Start camera node
            self.camera_process = subprocess.Popen([
                'ros2', 'run', 'camera_interface', 'camera_node'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            # Wait a moment for camera to initialize
            import time
            time.sleep(2)
            
            # Start hand pose node
            self.hand_pose_process = subprocess.Popen([
                'ros2', 'run', 'perception', 'hand_pose_node'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            self.perception_active = True
            self.get_logger().info('✅ Perception nodes started successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start perception nodes: {e}')
            self.stop_perception_nodes()
    
    def stop_perception_nodes(self):
        """Stop camera and hand pose nodes."""
        self.get_logger().info('🛑 Stopping perception nodes...')
        
        # Stop hand pose node
        if self.hand_pose_process:
            try:
                self.hand_pose_process.terminate()
                self.hand_pose_process.wait(timeout=5)
                self.get_logger().info('Hand pose node stopped')
            except subprocess.TimeoutExpired:
                self.hand_pose_process.kill()
                self.get_logger().warn('Hand pose node force killed')
            except Exception as e:
                self.get_logger().error(f'Error stopping hand pose node: {e}')
            finally:
                self.hand_pose_process = None
        
        # Stop camera node
        if self.camera_process:
            try:
                self.camera_process.terminate()
                self.camera_process.wait(timeout=5)
                self.get_logger().info('Camera node stopped')
            except subprocess.TimeoutExpired:
                self.camera_process.kill()
                self.get_logger().warn('Camera node force killed')
            except Exception as e:
                self.get_logger().error(f'Error stopping camera node: {e}')
            finally:
                self.camera_process = None
        
        # Also kill any remaining processes by name
        try:
            subprocess.run(['pkill', '-f', 'camera_node'], check=False)
            subprocess.run(['pkill', '-f', 'hand_pose_node'], check=False)
        except Exception:
            pass
        
        self.perception_active = False
        self.get_logger().info('✅ Perception nodes stopped')
    
    def publish_status(self):
        """Publish perception control status."""
        status_info = {
            'perception_active': self.perception_active,
            'current_mode': self.current_mode,
            'camera_running': self.camera_process is not None and self.camera_process.poll() is None,
            'hand_pose_running': self.hand_pose_process is not None and self.hand_pose_process.poll() is None,
            'node_type': 'perception_control'
        }
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.perception_status_pub.publish(status_msg)
    
    def shutdown_node(self):
        """Shutdown the node and stop all perception processes."""
        self.get_logger().info('Shutting down perception control node...')
        self.stop_perception_nodes()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Perception control node stopped by user')
    finally:
        node.shutdown_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
