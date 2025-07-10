#!/usr/bin/env python3
"""
Enhanced Lifecycle Action Server

Implements action-based lifecycle management following ROS2 best practices.
Supports multiple node transitions with comprehensive error handling and feedback.
"""

import rclpy
import time
import asyncio
from typing import List, Dict, Tuple
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State, Transition
from action_interfaces.action import LifecycleManagement


class LifecycleActionServer(Node):
    """
    Enhanced action server for lifecycle management.
    
    Provides robust, concurrent lifecycle transitions with:
    - Multiple node support
    - Comprehensive error handling
    - Real-time feedback
    - Cancellation support
    - Timeout management
    """
    
    def __init__(self):
        super().__init__('lifecycle_action_server')
        
        # Use reentrant callback group for concurrent operations
        self._cb_group = ReentrantCallbackGroup()
        
        # Create action server with proper callbacks
        self._action_server = ActionServer(self, LifecycleManagement, 'lifecycle_management')
        
        # Service clients for lifecycle transitions (created dynamically)
        self._service_clients: Dict[str, object] = {}
        
        # Configuration parameters
        self._managed_nodes = self.declare_parameter('managed_nodes', [
            'servo_interface_lifecycle_node',
            'camera_lifecycle_node',
            'hand_pose_node',
            'cr3_controller_node'
        ]).value
        self._default_timeout = self.declare_parameter('default_timeout', 10.0).value
        self._max_concurrent_transitions = self.declare_parameter('max_concurrent_transitions', 5).value
        
        # Track active goals for monitoring
        self._active_goals = {}
        
        self.get_logger().info(f'Lifecycle Action Server started')
        self.get_logger().info(f'Managing nodes: {self._managed_nodes}')
        self.get_logger().info(f'Default timeout: {self._default_timeout}s')
    
    def goal_callback(self, goal_request) -> GoalResponse:
        """
        Validate incoming goal requests.
        
        Args:
            goal_request: The incoming goal request
            
        Returns:
            GoalResponse: ACCEPT or REJECT
        """
        try:
            # Validate node names
            if not goal_request.node_names:
                self.get_logger().warn('Goal rejected: No node names provided')
                return GoalResponse.REJECT
            
            if len(goal_request.node_names) > self._max_concurrent_transitions:
                self.get_logger().warn(
                    f'Goal rejected: Too many nodes ({len(goal_request.node_names)} > {self._max_concurrent_transitions})'
                )
                return GoalResponse.REJECT
            
            # Check if nodes are in managed list (if configured)
            if self._managed_nodes:
                for node_name in goal_request.node_names:
                    if node_name not in self._managed_nodes:
                        self.get_logger().warn(f'Goal rejected: {node_name} not in managed nodes list')
                        return GoalResponse.REJECT
            
            # Validate transition
            valid_transitions = [
                Transition.TRANSITION_CONFIGURE,      # 1
                Transition.TRANSITION_ACTIVATE,       # 2
                Transition.TRANSITION_DEACTIVATE,     # 3
                Transition.TRANSITION_CLEANUP,        # 4
                Transition.TRANSITION_SHUTDOWN        # 5
            ]
            
            if goal_request.transition not in valid_transitions:
                self.get_logger().warn(f'Goal rejected: Invalid transition {goal_request.transition}')
                return GoalResponse.REJECT
            
            # Validate timeout
            if goal_request.timeout < 0:
                self.get_logger().warn('Goal rejected: Negative timeout not allowed')
                return GoalResponse.REJECT
            
            self.get_logger().info(
                f'Goal accepted: {len(goal_request.node_names)} nodes, '
                f'transition={goal_request.transition}, timeout={goal_request.timeout}s'
            )
            return GoalResponse.ACCEPT
            
        except Exception as e:
            self.get_logger().error(f'Goal validation failed: {str(e)}')
            return GoalResponse.REJECT
    
    def cancel_callback(self, goal_handle) -> CancelResponse:
        """
        Handle goal cancellation requests.
        
        Args:
            goal_handle: The goal handle to cancel
            
        Returns:
            CancelResponse: ACCEPT (we always allow cancellation)
        """
        self.get_logger().info(f'Cancellation requested for goal {goal_handle.goal_id}')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """
        Execute lifecycle transitions with comprehensive error handling.
        
        Args:
            goal_handle: The goal handle containing the request
            
        Returns:
            LifecycleManagement.Result: The execution result
        """
        goal = goal_handle.request
        feedback_msg = LifecycleManagement.Feedback()
        result = LifecycleManagement.Result()
        start_time = time.time()
        
        # Store goal for monitoring
        self._active_goals[goal_handle.goal_id] = {
            'start_time': start_time,
            'nodes': goal.node_names,
            'transition': goal.transition
        }
        
        try:
            # Initialize result tracking
            total_nodes = len(goal.node_names)
            timeout = goal.timeout if goal.timeout > 0 else self._default_timeout
            
            # Initialize result arrays
            result.node_names = goal.node_names
            result.node_success = [False] * total_nodes
            result.node_messages = [''] * total_nodes
            result.node_durations = [0.0] * total_nodes
            
            self.get_logger().info(
                f'Starting transition {self._transition_name(goal.transition)} for {total_nodes} nodes '
                f'(timeout: {timeout}s, wait_for_completion: {goal.wait_for_completion})'
            )
            
            # Initial feedback
            feedback_msg.total_nodes = total_nodes
            feedback_msg.nodes_completed = 0
            feedback_msg.progress_percentage = 0.0
            feedback_msg.status_message = f'Starting transition for {total_nodes} nodes...'
            goal_handle.publish_feedback(feedback_msg)
            
            # Execute transitions for each node
            completed_nodes = 0
            overall_success = True
            
            for i, node_name in enumerate(goal.node_names):
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = f'Operation cancelled after {completed_nodes}/{total_nodes} nodes'
                    result.total_duration = time.time() - start_time
                    return result
                
                # Update feedback for current node
                feedback_msg.current_node = node_name
                feedback_msg.nodes_completed = completed_nodes
                feedback_msg.progress_percentage = (i / total_nodes) * 100.0
                feedback_msg.status_message = f'Processing {node_name}...'
                goal_handle.publish_feedback(feedback_msg)
                
                # Execute transition for this node
                node_start_time = time.time()
                success, message = await self._execute_node_transition(
                    node_name, goal.transition, timeout
                )
                node_duration = time.time() - node_start_time
                
                # Store results
                result.node_success[i] = success
                result.node_messages[i] = message
                result.node_durations[i] = node_duration
                
                if success:
                    completed_nodes += 1
                    self.get_logger().info(f'✓ {node_name}: {message} ({node_duration:.2f}s)')
                else:
                    overall_success = False
                    self.get_logger().error(f'✗ {node_name}: {message} ({node_duration:.2f}s)')
                    
                    # Stop on first failure if not waiting for completion
                    if not goal.wait_for_completion:
                        self.get_logger().warn('Stopping on first failure (wait_for_completion=False)')
                        break
                
                # Update progress
                feedback_msg.nodes_completed = completed_nodes
                feedback_msg.progress_percentage = ((i + 1) / total_nodes) * 100.0
                goal_handle.publish_feedback(feedback_msg)
            
            # Final feedback
            feedback_msg.current_node = ''
            feedback_msg.nodes_completed = completed_nodes
            feedback_msg.progress_percentage = 100.0
            feedback_msg.status_message = 'Completed'
            goal_handle.publish_feedback(feedback_msg)
            
            # Set final result
            result.success = overall_success
            result.total_duration = time.time() - start_time
            
            if overall_success:
                result.message = f'Successfully transitioned all {completed_nodes} nodes'
                goal_handle.succeed()
                self.get_logger().info(f'✅ Goal completed successfully: {result.message} ({result.total_duration:.2f}s)')
            else:
                failed_count = total_nodes - completed_nodes
                result.message = f'Failed to transition {failed_count}/{total_nodes} nodes'
                goal_handle.abort()
                self.get_logger().error(f'❌ Goal aborted: {result.message} ({result.total_duration:.2f}s)')
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'Unexpected error during execution: {str(e)}')
            goal_handle.abort()
            result.success = False
            result.message = f'Internal error: {str(e)}'
            result.total_duration = time.time() - start_time
            return result
            
        finally:
            # Clean up goal tracking
            if goal_handle.goal_id in self._active_goals:
                del self._active_goals[goal_handle.goal_id]
    
    async def _execute_node_transition(self, node_name: str, transition: int, timeout: float) -> Tuple[bool, str]:
        """
        Execute transition for a single node with timeout and error handling.
        
        Args:
            node_name: Name of the node to transition
            transition: Transition ID
            timeout: Maximum time to wait
            
        Returns:
            Tuple[bool, str]: (success, message)
        """
        try:
            # Get or create service client
            client = self._get_service_client(node_name)
            
            # Wait for service with timeout
            service_timeout = min(timeout, 5.0)  # Max 5 seconds for service discovery
            if not client.wait_for_service(timeout_sec=service_timeout):
                return False, f'Service {client.srv_name} not available after {service_timeout}s'
            
            # Create and send request
            request = ChangeState.Request()
            request.transition = Transition()
            request.transition.id = transition
            
            # Call service asynchronously
            future = client.call_async(request)
            
            # Wait for response with timeout
            try:
                await asyncio.wait_for(
                    asyncio.wrap_future(future),
                    timeout=timeout
                )
            except asyncio.TimeoutError:
                return False, f'Transition timeout after {timeout}s'
            
            # Process response
            response = future.result()
            if response.success:
                return True, f'Transition to {self._transition_name(transition)} succeeded'
            else:
                return False, 'Transition failed (service returned failure)'
                
        except Exception as e:
            return False, f'Exception during transition: {str(e)}'
    
    def _get_service_client(self, node_name: str):
        """
        Get or create service client for the given node.
        
        Args:
            node_name: Name of the lifecycle node
            
        Returns:
            Service client for the node's change_state service
        """
        service_name = f'/{node_name}/change_state'
        
        if service_name not in self._service_clients:
            self.get_logger().debug(f'Creating service client for {service_name}')
            self._service_clients[service_name] = self.create_client(
                ChangeState,
                service_name,
                callback_group=self._cb_group
            )
        
        return self._service_clients[service_name]
    
    def _transition_name(self, transition_id: int) -> str:
        """
        Convert transition ID to human-readable name.
        
        Args:
            transition_id: Transition ID
            
        Returns:
            Human-readable transition name
        """
        transition_names = {
            Transition.TRANSITION_CONFIGURE: 'configure',
            Transition.TRANSITION_ACTIVATE: 'activate',
            Transition.TRANSITION_DEACTIVATE: 'deactivate',
            Transition.TRANSITION_CLEANUP: 'cleanup',
            Transition.TRANSITION_SHUTDOWN: 'shutdown'
        }
        return transition_names.get(transition_id, f'unknown({transition_id})')
    
    def get_active_goals_info(self) -> Dict:
        """
        Get information about currently active goals.
        
        Returns:
            Dictionary with active goal information
        """
        return {
            goal_id: {
                'duration': time.time() - info['start_time'],
                'nodes': info['nodes'],
                'transition': self._transition_name(info['transition'])
            }
            for goal_id, info in self._active_goals.items()
        }


def main(args=None):
    """Main entry point for the lifecycle action server."""
    rclpy.init(args=args)
    
    # Create action server
    action_server = LifecycleActionServer()
    
    # Use MultiThreadedExecutor for concurrent action handling
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(action_server)
    
    try:
        action_server.get_logger().info('Lifecycle Action Server spinning...')
        executor.spin()
    except KeyboardInterrupt:
        action_server.get_logger().info('Shutting down Lifecycle Action Server')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
