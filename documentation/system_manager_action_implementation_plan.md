# System Manager Action-Based Implementation Plan

## Overview

This document outlines the detailed implementation plan for refactoring the system_manager package to use action-based lifecycle management following ROS2 best practices. Based on our research of official ROS2 documentation, this plan provides a roadmap for robust, scalable lifecycle management.

## Current State Analysis

### What We Have
- ✅ `system_manager` package with hybrid CMake/Python structure
- ✅ `LifecycleManagement.action` interface defined
- ✅ Draft `lifecycle_action_server.py` implementation
- ✅ Legacy `lifecycle_manager.py` (service-based)
- ✅ `mode_switcher_node.py` for mode management
- ✅ Package builds successfully with action interface available

### What Needs Improvement
- 🔄 Action server implementation needs to follow best practices
- 🔄 Better error handling and goal validation
- 🔄 Proper feedback and result reporting
- 🔄 Integration with mode_switcher_node
- 🔄 Comprehensive testing
- 🔄 Documentation and usage examples

## Action Interface Analysis

### Current LifecycleManagement.action
```
# Goal: Request to transition a node
string node_name
uint8 transition
---
# Result: Outcome of the transition
bool success
string message
---
# Feedback: Progress updates
string status
```

### Improvements Needed

1. **Enhanced Goal Definition**
   ```
   # Goal: Request to transition one or more nodes
   string[] node_names              # Support multiple nodes
   uint8 transition                 # Transition ID (configure, activate, etc.)
   float64 timeout                  # Maximum time to wait for completion
   bool wait_for_completion         # Whether to wait for all nodes
   ---
   # Result: Final outcome
   bool success                     # Overall success
   string message                   # Summary message
   NodeTransitionResult[] results   # Per-node results
   ---
   # Feedback: Real-time progress
   string current_node              # Currently processing node
   uint8 nodes_completed           # Number of nodes completed
   uint8 total_nodes               # Total number of nodes
   string status                   # Current operation status
   ```

2. **Additional Message Types**
   ```
   # NodeTransitionResult.msg
   string node_name
   bool success
   string error_message
   float64 duration
   ```

## Implementation Plan

### Phase 1: Core Action Server Enhancement

#### 1.1 Improved Action Server Structure
```python
import rclpy
import time
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import State, Transition
from system_manager_interfaces.action import LifecycleManagement

class LifecycleActionServer(Node):
    def __init__(self):
        super().__init__('lifecycle_action_server')
        
        # Use reentrant callback group for concurrent operations
        self._cb_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            LifecycleManagement,
            'lifecycle_management',
            self.execute_callback,
            callback_group=self._cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Track active goals and service clients
        self._active_goals = {}
        self._service_clients = {}
        
        # Node configuration
        self._managed_nodes = self.declare_parameter('managed_nodes', []).value
        self._default_timeout = self.declare_parameter('default_timeout', 10.0).value
        
        self.get_logger().info('Lifecycle Action Server started')
```

#### 1.2 Goal Validation and Handling
```python
def goal_callback(self, goal_request):
    """Validate incoming goal requests."""
    try:
        # Validate node names
        if not goal_request.node_names:
            self.get_logger().warn('Goal rejected: No node names provided')
            return GoalResponse.REJECT
        
        # Check if nodes are managed
        for node_name in goal_request.node_names:
            if node_name not in self._managed_nodes:
                self.get_logger().warn(f'Goal rejected: {node_name} not in managed nodes')
                return GoalResponse.REJECT
        
        # Validate transition
        valid_transitions = [
            Transition.TRANSITION_CONFIGURE,
            Transition.TRANSITION_ACTIVATE,
            Transition.TRANSITION_DEACTIVATE,
            Transition.TRANSITION_CLEANUP,
            Transition.TRANSITION_SHUTDOWN
        ]
        
        if goal_request.transition not in valid_transitions:
            self.get_logger().warn(f'Goal rejected: Invalid transition {goal_request.transition}')
            return GoalResponse.REJECT
        
        self.get_logger().info(f'Goal accepted for nodes: {goal_request.node_names}')
        return GoalResponse.ACCEPT
        
    except Exception as e:
        self.get_logger().error(f'Goal validation failed: {str(e)}')
        return GoalResponse.REJECT

def cancel_callback(self, goal_handle):
    """Handle goal cancellation requests."""
    self.get_logger().info(f'Cancellation requested for goal {goal_handle.goal_id}')
    return CancelResponse.ACCEPT
```

#### 1.3 Enhanced Execute Callback
```python
def execute_callback(self, goal_handle):
    """Execute lifecycle transitions with comprehensive error handling."""
    goal = goal_handle.request
    feedback_msg = LifecycleManagement.Feedback()
    result = LifecycleManagement.Result()
    
    try:
        # Initialize result tracking
        results = []
        total_nodes = len(goal.node_names)
        completed_nodes = 0
        overall_success = True
        
        # Set timeout
        timeout = goal.timeout if goal.timeout > 0 else self._default_timeout
        
        self.get_logger().info(
            f'Starting transition {goal.transition} for {total_nodes} nodes '
            f'(timeout: {timeout}s)'
        )
        
        # Process each node
        for i, node_name in enumerate(goal.node_names):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Operation cancelled by user'
                result.results = results
                return result
            
            # Update feedback
            feedback_msg.current_node = node_name
            feedback_msg.nodes_completed = completed_nodes
            feedback_msg.total_nodes = total_nodes
            feedback_msg.status = f'Processing {node_name}...'
            goal_handle.publish_feedback(feedback_msg)
            
            # Execute transition
            node_result = self._execute_node_transition(
                node_name, goal.transition, timeout
            )
            results.append(node_result)
            
            if node_result.success:
                completed_nodes += 1
                self.get_logger().info(f'✓ {node_name}: {node_result.message}')
            else:
                overall_success = False
                self.get_logger().error(f'✗ {node_name}: {node_result.error_message}')
                
                # Stop on first failure if not waiting for completion
                if not goal.wait_for_completion:
                    break
        
        # Final feedback
        feedback_msg.nodes_completed = completed_nodes
        feedback_msg.status = 'Completed'
        goal_handle.publish_feedback(feedback_msg)
        
        # Set result
        result.success = overall_success
        result.results = results
        
        if overall_success:
            result.message = f'Successfully transitioned {completed_nodes}/{total_nodes} nodes'
            goal_handle.succeed()
            self.get_logger().info(f'Goal completed successfully: {result.message}')
        else:
            result.message = f'Failed to transition {total_nodes - completed_nodes}/{total_nodes} nodes'
            goal_handle.abort()
            self.get_logger().error(f'Goal aborted: {result.message}')
        
        return result
        
    except Exception as e:
        self.get_logger().error(f'Unexpected error during execution: {str(e)}')
        goal_handle.abort()
        result.success = False
        result.message = f'Internal error: {str(e)}'
        return result
```

### Phase 2: Service Client Management

#### 2.1 Dynamic Service Client Creation
```python
def _ensure_service_client(self, node_name):
    """Ensure service client exists for the given node."""
    service_name = f'/{node_name}/change_state'
    
    if service_name not in self._service_clients:
        self.get_logger().info(f'Creating service client for {service_name}')
        self._service_clients[service_name] = self.create_client(
            ChangeState,
            service_name,
            callback_group=self._cb_group
        )
    
    return self._service_clients[service_name]

def _execute_node_transition(self, node_name, transition, timeout):
    """Execute transition for a single node with timeout."""
    start_time = time.time()
    
    try:
        # Get or create service client
        client = self._ensure_service_client(node_name)
        
        # Wait for service with timeout
        if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
            return NodeTransitionResult(
                node_name=node_name,
                success=False,
                error_message=f'Service {client.srv_name} not available',
                duration=time.time() - start_time
            )
        
        # Create request
        request = ChangeState.Request()
        request.transition = Transition()
        request.transition.id = transition
        
        # Call service with timeout
        future = client.call_async(request)
        remaining_timeout = timeout - (time.time() - start_time)
        
        if not self._wait_for_future(future, remaining_timeout):
            return NodeTransitionResult(
                node_name=node_name,
                success=False,
                error_message=f'Transition timeout after {timeout}s',
                duration=time.time() - start_time
            )
        
        # Process response
        response = future.result()
        duration = time.time() - start_time
        
        if response.success:
            return NodeTransitionResult(
                node_name=node_name,
                success=True,
                error_message='',
                duration=duration
            )
        else:
            return NodeTransitionResult(
                node_name=node_name,
                success=False,
                error_message='Transition failed (service returned failure)',
                duration=duration
            )
            
    except Exception as e:
        return NodeTransitionResult(
            node_name=node_name,
            success=False,
            error_message=f'Exception during transition: {str(e)}',
            duration=time.time() - start_time
        )
```

### Phase 3: Mode Switcher Integration

#### 3.1 Enhanced Mode Switcher
```python
from rclpy.action import ActionClient

class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')
        
        # Action client for lifecycle management
        self._lifecycle_client = ActionClient(
            self,
            LifecycleManagement,
            'lifecycle_management'
        )
        
        # Mode definitions
        self._modes = {
            'idle': {
                'nodes_to_activate': [],
                'nodes_to_deactivate': ['camera_node', 'hand_pose_node', 'cr3_controller_node']
            },
            'manual': {
                'nodes_to_activate': ['camera_node', 'cr3_controller_node'],
                'nodes_to_deactivate': ['hand_pose_node', 'trajectory_executor_node']
            },
            'autonomous': {
                'nodes_to_activate': ['camera_node', 'hand_pose_node', 'trajectory_executor_node'],
                'nodes_to_deactivate': []
            }
        }
        
        self.get_logger().info('Mode Switcher ready')
    
    async def switch_mode(self, mode_name):
        """Switch to specified mode using action-based lifecycle management."""
        if mode_name not in self._modes:
            self.get_logger().error(f'Unknown mode: {mode_name}')
            return False
        
        mode_config = self._modes[mode_name]
        
        try:
            # Deactivate nodes first
            if mode_config['nodes_to_deactivate']:
                success = await self._transition_nodes(
                    mode_config['nodes_to_deactivate'],
                    Transition.TRANSITION_DEACTIVATE
                )
                if not success:
                    return False
            
            # Then activate required nodes
            if mode_config['nodes_to_activate']:
                success = await self._transition_nodes(
                    mode_config['nodes_to_activate'],
                    Transition.TRANSITION_ACTIVATE
                )
                if not success:
                    return False
            
            self.get_logger().info(f'Successfully switched to mode: {mode_name}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Mode switch failed: {str(e)}')
            return False
    
    async def _transition_nodes(self, node_names, transition):
        """Use action client to transition nodes."""
        if not self._lifecycle_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Lifecycle action server not available')
            return False
        
        # Create goal
        goal = LifecycleManagement.Goal()
        goal.node_names = node_names
        goal.transition = transition
        goal.timeout = 30.0
        goal.wait_for_completion = True
        
        # Send goal and wait for result
        future = self._lifecycle_client.send_goal_async(goal)
        goal_handle = await future
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by lifecycle action server')
            return False
        
        result_future = goal_handle.get_result_async()
        result = await result_future
        
        return result.result.success
```

### Phase 4: Testing and Validation

#### 4.1 Unit Tests
```python
import unittest
from unittest.mock import Mock, patch
import rclpy
from system_manager.lifecycle_action_server import LifecycleActionServer

class TestLifecycleActionServer(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = LifecycleActionServer()
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_goal_validation_valid_request(self):
        # Test valid goal acceptance
        pass
    
    def test_goal_validation_invalid_nodes(self):
        # Test rejection of unknown nodes
        pass
    
    def test_transition_execution_success(self):
        # Test successful node transition
        pass
    
    def test_transition_execution_timeout(self):
        # Test timeout handling
        pass
```

#### 4.2 Integration Tests
- Action server/client communication
- Mode switching scenarios
- Error recovery
- Concurrent goal handling

### Phase 5: Documentation and Deployment

#### 5.1 Usage Documentation
- API reference for action interface
- Mode switching examples
- Troubleshooting guide
- Performance tuning

#### 5.2 Launch Files
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_manager',
            executable='lifecycle_action_server',
            name='lifecycle_action_server',
            parameters=[{
                'managed_nodes': [
                    'camera_node',
                    'hand_pose_node', 
                    'cr3_controller_node',
                    'trajectory_executor_node'
                ],
                'default_timeout': 15.0
            }]
        ),
        Node(
            package='system_manager',
            executable='mode_switcher',
            name='mode_switcher'
        )
    ])
```

## Migration Strategy

### Step 1: Parallel Implementation
- Keep existing service-based lifecycle_manager
- Implement new action-based server alongside
- Test both systems in parallel

### Step 2: Gradual Migration
- Update mode_switcher to use action client
- Update launch files to use new action server
- Maintain backward compatibility

### Step 3: Complete Transition
- Remove legacy service-based implementation
- Update all documentation
- Deploy to production

## Success Criteria

- ✅ Action server handles multiple concurrent goals
- ✅ Comprehensive error handling and recovery
- ✅ Real-time feedback during operations
- ✅ Mode switching works reliably
- ✅ System performance meets requirements
- ✅ Full test coverage (unit + integration)
- ✅ Complete documentation

## Timeline

- **Week 1**: Core action server implementation
- **Week 2**: Mode switcher integration and testing
- **Week 3**: Documentation and deployment
- **Week 4**: Performance tuning and optimization

This plan provides a structured approach to implementing robust, action-based lifecycle management following ROS2 best practices while ensuring system reliability and maintainability.
