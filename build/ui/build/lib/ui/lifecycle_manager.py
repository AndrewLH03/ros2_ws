#!/usr/bin/env python3
"""
Lifecycle Manager for CR3 Control System

This node manages the lifecycle states of all system nodes and handles
mode switching between manual, perception, simulation, and diagnostic modes.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import TransitionEvent, Transition
from std_msgs.msg import String
import time
from threading import Lock

class LifecycleManager(Node):
    """Central manager for lifecycle nodes and mode switching."""
    
    def __init__(self):
        super().__init__('lifecycle_manager')
        
        # Current system mode
        self.current_mode = 'manual'
        self.mode_lock = Lock()
        
        # Define node groups for different modes
        self.node_groups = {
            'core': [
                'servo_interface_node',           # Core servo control
                'emergency_stop_node',            # Critical safety
                'ui_dashboard_node',              # Primary user interface
                'mode_switcher_node',             # Mode switching UI
                'cr3_controller_node',            # Core robot controller
                'joint_state_publisher_node',     # Essential robot state
                'tf_broadcaster_node'             # Core coordinate frames
            ],
            'manual': [
                'manual_control_node',            # Manual control logic
                'manual_servo_control_node',      # Direct servo commands
                'teleop_node',                    # Teleoperation interface
                'direct_servo_tester_node'        # Manual servo testing
            ],
            'perception': [
                'camera_node',                    # Camera input
                'camera_info_node',               # Camera calibration
                'hand_pose_node',                 # Hand detection
                'body_pose_node',                 # Body detection
                'perception_control_node',        # Perception-based control
                'coordinate_transform_node',      # Coordinate transformations
                'pose_filter_node'                # Pose filtering
            ],
            'simulation': [
                'simulator_node',                 # Simulation environment
                'sim_world_interface_node'        # Simulation interface
            ],
            'diagnostic': [
                'health_monitor_node',            # System health monitoring
                'logger_node',                    # System logging
                'error_handler_node',             # Error handling
                'watchdog_node',                  # System watchdog
                'network_monitor_node',           # Network diagnostics
                'resource_monitor_node',          # Resource monitoring
                'system_metrics_node'             # System metrics
            ],
            'cr3_interface': [
                'hand_controller_node',           # CR3 hand control
                'ip_to_ros_bridge_node',          # Network bridge
                'motion_planner_node',            # Motion planning
                'pose_to_command_node',           # Pose to command conversion
                'trajectory_executor_node'        # Trajectory execution
            ]
        }
        
        # Track node states
        self.node_states = {}
        
        # Publishers and Subscribers
        self.mode_pub = self.create_publisher(String, '/current_mode', 10)
        self.mode_sub = self.create_subscription(
            String, '/mode_request', self.handle_mode_change, 10)
        
        # Service clients for lifecycle management
        self.lifecycle_clients = {}
        self.state_clients = {}
        
        # Initialize clients for all managed nodes
        self._initialize_clients()
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        # Configure and activate core nodes on startup
        self.create_timer(2.0, self.startup_sequence)
        
        self.get_logger().info('Lifecycle Manager initialized')
    
    def _initialize_clients(self):
        """Initialize service clients for all managed nodes."""
        all_nodes = []
        for group_nodes in self.node_groups.values():
            all_nodes.extend(group_nodes)
        
        for node_name in set(all_nodes):  # Remove duplicates
            # Change state service client
            self.lifecycle_clients[node_name] = self.create_client(
                ChangeState, f'/{node_name}/change_state')
            
            # Get state service client
            self.state_clients[node_name] = self.create_client(
                GetState, f'/{node_name}/get_state')
            
            self.node_states[node_name] = 'unknown'
    
    def startup_sequence(self):
        """Configure and activate core nodes on startup."""
        self.get_logger().info('Starting core node configuration sequence')
        
        # Configure core nodes
        for node_name in self.node_groups['core']:
            self.configure_node(node_name)
            time.sleep(0.5)  # Small delay between configurations
        
        # Activate core nodes
        time.sleep(1.0)
        for node_name in self.node_groups['core']:
            self.activate_node(node_name)
            time.sleep(0.5)
        
        # Start in manual mode
        self.activate_mode('manual')
        
        # Cancel the timer
        self.startup_timer.cancel() if hasattr(self, 'startup_timer') else None
    
    def handle_mode_change(self, msg):
        """Handle mode change requests."""
        new_mode = msg.data
        
        if new_mode not in self.node_groups:
            self.get_logger().error(f'Unknown mode: {new_mode}')
            return
        
        if new_mode == self.current_mode:
            self.get_logger().info(f'Already in {new_mode} mode')
            return
        
        with self.mode_lock:
            self.get_logger().info(f'Switching from {self.current_mode} to {new_mode} mode')
            
            # Deactivate current mode nodes
            if self.current_mode in self.node_groups:
                self.deactivate_mode(self.current_mode)
            
            # Small delay for clean transition
            time.sleep(0.5)
            
            # Activate new mode nodes
            self.activate_mode(new_mode)
            
            self.current_mode = new_mode
            
            # Publish current mode
            mode_msg = String()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)
            
            self.get_logger().info(f'Successfully switched to {new_mode} mode')
    
    def activate_mode(self, mode):
        """Activate all nodes for a specific mode."""
        if mode not in self.node_groups:
            self.get_logger().error(f'Unknown mode: {mode}')
            return
        
        nodes = self.node_groups[mode]
        self.get_logger().info(f'Activating {len(nodes)} nodes for {mode} mode')
        
        # Configure nodes first
        for node_name in nodes:
            success = self.configure_node(node_name)
            if success:
                time.sleep(0.2)  # Small delay between configurations
        
        # Then activate them
        time.sleep(0.5)
        for node_name in nodes:
            success = self.activate_node(node_name)
            if success:
                time.sleep(0.2)  # Small delay between activations
    
    def deactivate_mode(self, mode):
        """Deactivate all nodes for a specific mode."""
        if mode not in self.node_groups:
            return
        
        nodes = self.node_groups[mode]
        self.get_logger().info(f'Deactivating {len(nodes)} nodes for {mode} mode')
        
        for node_name in nodes:
            self.deactivate_node(node_name)
            time.sleep(0.1)  # Small delay between deactivations
    
    def configure_node(self, node_name):
        """Configure a specific node."""
        return self._change_node_state(node_name, Transition.TRANSITION_CONFIGURE)
    
    def activate_node(self, node_name):
        """Activate a specific node."""
        return self._change_node_state(node_name, Transition.TRANSITION_ACTIVATE)
    
    def deactivate_node(self, node_name):
        """Deactivate a specific node."""
        return self._change_node_state(node_name, Transition.TRANSITION_DEACTIVATE)
    
    def cleanup_node(self, node_name):
        """Cleanup a specific node."""
        return self._change_node_state(node_name, Transition.TRANSITION_CLEANUP)
    
    def _change_node_state(self, node_name, transition_id):
        """Change the state of a specific node."""
        if node_name not in self.lifecycle_clients:
            self.get_logger().warning(f'No client for node: {node_name}')
            return False
        
        client = self.lifecycle_clients[node_name]
        
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(f'Service for {node_name} not available')
            return False
        
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    transition_name = self._get_transition_name(transition_id)
                    self.get_logger().debug(f'{node_name}: {transition_name} successful')
                    return True
                else:
                    transition_name = self._get_transition_name(transition_id)
                    self.get_logger().error(f'{node_name}: {transition_name} failed')
                    return False
            else:
                self.get_logger().error(f'Service call to {node_name} timed out')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error calling service for {node_name}: {e}')
            return False
    
    def _get_transition_name(self, transition_id):
        """Get human-readable transition name."""
        transition_names = {
            Transition.TRANSITION_CONFIGURE: 'configure',
            Transition.TRANSITION_ACTIVATE: 'activate',
            Transition.TRANSITION_DEACTIVATE: 'deactivate',
            Transition.TRANSITION_CLEANUP: 'cleanup',
            Transition.TRANSITION_SHUTDOWN: 'shutdown'
        }
        return transition_names.get(transition_id, 'unknown')
    
    def get_node_state(self, node_name):
        """Get the current state of a node."""
        if node_name not in self.state_clients:
            return 'unknown'
        
        client = self.state_clients[node_name]
        
        if not client.wait_for_service(timeout_sec=1.0):
            return 'unavailable'
        
        request = GetState.Request()
        
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                return response.current_state.label
            else:
                return 'timeout'
                
        except Exception as e:
            self.get_logger().debug(f'Error getting state for {node_name}: {e}')
            return 'error'
    
    def publish_status(self):
        """Publish current mode and update node states."""
        # Update node states
        for node_name in self.node_states.keys():
            self.node_states[node_name] = self.get_node_state(node_name)
        
        # Publish current mode
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)
        
        # Log status every 30 seconds
        if hasattr(self, '_last_status_log'):
            if time.time() - self._last_status_log > 30.0:
                self._log_system_status()
                self._last_status_log = time.time()
        else:
            self._last_status_log = time.time()
    
    def _log_system_status(self):
        """Log current system status."""
        active_nodes = [name for name, state in self.node_states.items() 
                       if state == 'active']
        self.get_logger().info(
            f'System Status - Mode: {self.current_mode}, '
            f'Active nodes: {len(active_nodes)}'
        )
    
    def emergency_shutdown(self):
        """Emergency shutdown of all nodes."""
        self.get_logger().warning('Emergency shutdown initiated')
        
        with self.mode_lock:
            # Deactivate all mode-specific nodes
            for mode in self.node_groups:
                if mode != 'core':
                    self.deactivate_mode(mode)
            
            # Deactivate core nodes last
            self.deactivate_mode('core')
            
            self.current_mode = 'shutdown'

def main(args=None):
    rclpy.init(args=args)
    
    try:
        lifecycle_manager = LifecycleManager()
        
        # Use MultiThreadedExecutor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(lifecycle_manager)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            lifecycle_manager.get_logger().info('Keyboard interrupt received')
            lifecycle_manager.emergency_shutdown()
        
    except Exception as e:
        print(f'Error starting lifecycle manager: {e}')
    finally:
        try:
            lifecycle_manager.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
