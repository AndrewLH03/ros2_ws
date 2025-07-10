# Control System Lifecycle Node Conversion Plan

## Overview
This document outlines the detailed plan for converting all nodes in the `control_system` package from regular ROS2 nodes to lifecycle nodes. This conversion will enable proper state management, controlled startup/shutdown sequences, and robust error handling.

## Current Package Structure
```
control_system/
├── control_system/
│   ├── __init__.py
│   ├── servo_interface_node.py           # → servo_interface_lifecycle_node.py
│   ├── manual_control_node.py            # → manual_control_lifecycle_node.py
│   ├── perception_control_node.py        # → perception_control_lifecycle_node.py
│   ├── unified_control_node.py           # → unified_control_lifecycle_node.py
│   ├── mode_manager_node.py              # → mode_manager_lifecycle_node.py
│   └── servo_interface_lifecycle_node.py # ✓ Already exists
├── package.xml                           # ✓ Updated with lifecycle dependencies
└── setup.py                             # → Update entry points
```

## Lifecycle Node States and Transitions

### State Machine
```
Unconfigured ──configure()──► Inactive ──activate()──► Active
     ▲                           │                        │
     │                           │                        │
     └──cleanup()──◄─────────────┘                        │
                                                           │
Finalized ◄──shutdown()──◄─────────────────────────────────┘
```

### State Purposes
- **Unconfigured**: Node created, no resources allocated
- **Inactive**: Resources configured, not processing data
- **Active**: Fully operational, processing data
- **Finalized**: Shutdown complete, resources released

## Phase 1: Core Infrastructure Conversion

### 1.1 Servo Interface Node Conversion

#### Current Implementation Issues
```python
# servo_interface_node.py - Current approach
class ServoInterfaceNode(Node):
    def __init__(self):
        super().__init__('servo_interface_node')
        # ❌ Immediately tries to connect to hardware
        self.connect_to_servos()  # Fails if hardware not ready
        # ❌ Publishers always active
        self.position_pub = self.create_publisher(...)
```

#### Lifecycle Implementation
```python
# servo_interface_lifecycle_node.py - Target implementation
class ServoInterfaceLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('servo_interface_node')
        # ✓ Only declare variables, no hardware connection
        self.dynamixel_sdk = None
        self.servo_ports = None
        self.position_pub = None
        self.command_sub = None
        
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure hardware connections and ROS interfaces"""
        try:
            # Initialize Dynamixel SDK
            self.dynamixel_sdk = DynamixelSDK()
            
            # Test hardware connection (but don't enable motors)
            if not self.dynamixel_sdk.test_connection():
                self.get_logger().error("Failed to connect to Dynamixel hardware")
                return TransitionCallbackReturn.FAILURE
            
            # Create lifecycle publishers/subscribers
            self.position_pub = self.create_lifecycle_publisher(
                Float32MultiArray, '/servo/positions', 10)
            self.command_sub = self.create_subscription(
                Float32MultiArray, '/servo/commands', self.command_callback, 10)
            
            self.get_logger().info("Servo interface configured successfully")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Enable servo motors and start processing"""
        try:
            # Enable servo torque
            self.dynamixel_sdk.enable_torque()
            
            # Activate publisher
            self.position_pub.on_activate(state)
            
            # Start position monitoring timer
            self.position_timer = self.create_timer(0.01, self.publish_positions)
            
            self.get_logger().info("Servo interface activated - motors enabled")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Disable servo motors but keep connection"""
        try:
            # Stop position timer
            if hasattr(self, 'position_timer'):
                self.destroy_timer(self.position_timer)
            
            # Send stop command to servos
            self.dynamixel_sdk.stop_all_servos()
            
            # Disable servo torque for safety
            self.dynamixel_sdk.disable_torque()
            
            # Deactivate publisher
            self.position_pub.on_deactivate(state)
            
            self.get_logger().info("Servo interface deactivated - motors disabled")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Deactivation failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Close hardware connections and clean up resources"""
        try:
            # Destroy ROS interfaces
            if self.position_pub:
                self.destroy_lifecycle_publisher(self.position_pub)
            if self.command_sub:
                self.destroy_subscription(self.command_sub)
            
            # Close hardware connection
            if self.dynamixel_sdk:
                self.dynamixel_sdk.close_connection()
            
            self.get_logger().info("Servo interface cleaned up")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Cleanup failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def command_callback(self, msg):
        """Only process commands when active"""
        if self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.dynamixel_sdk.send_commands(msg.data)
```

### 1.2 Manual Control Node Conversion

#### Current Implementation
```python
# manual_control_node.py - Current approach
class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        # ❌ Always listening for commands
        self.command_sub = self.create_subscription(...)
        # ❌ Always publishing servo commands
        self.servo_pub = self.create_publisher(...)
```

#### Lifecycle Implementation
```python
# manual_control_lifecycle_node.py - Target implementation
class ManualControlLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('manual_control_node')
        self.command_sub = None
        self.servo_pub = None
        self.accepting_commands = False
        
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Set up ROS interfaces"""
        try:
            # Create command subscriber
            self.command_sub = self.create_subscription(
                String, '/manual/commands', self.command_callback, 10)
            
            # Create servo command publisher
            self.servo_pub = self.create_lifecycle_publisher(
                Float32MultiArray, '/servo/commands', 10)
            
            self.get_logger().info("Manual control configured")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start accepting manual commands"""
        try:
            # Activate publisher
            self.servo_pub.on_activate(state)
            
            # Enable command processing
            self.accepting_commands = True
            
            self.get_logger().info("Manual control activated - accepting commands")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop accepting commands and send stop command"""
        try:
            # Stop accepting new commands
            self.accepting_commands = False
            
            # Send stop command to servos
            stop_msg = Float32MultiArray()
            stop_msg.data = [0.0] * 5  # Stop all servos
            self.servo_pub.publish(stop_msg)
            
            # Deactivate publisher
            self.servo_pub.on_deactivate(state)
            
            self.get_logger().info("Manual control deactivated")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            return TransitionCallbackReturn.FAILURE
    
    def command_callback(self, msg):
        """Only process commands when active and accepting"""
        if (self.get_current_state().id == LifecycleState.PRIMARY_STATE_ACTIVE 
            and self.accepting_commands):
            self.process_manual_command(msg.data)
```

## Phase 2: Perception System Integration

### 2.1 Perception Control Node Conversion

#### Lifecycle Implementation
```python
# perception_control_lifecycle_node.py
class PerceptionControlLifecycleNode(LifecycleNode):
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Set up perception data subscriptions"""
        # Create subscribers for hand pose, confidence, etc.
        # Create servo command publisher
        # Initialize gesture recognition models
        
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start processing perception data"""
        # Activate servo command publisher
        # Start processing timers
        # Enable gesture-to-servo mapping
        
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop perception processing"""
        # Send stop commands to servos
        # Stop processing timers
        # Deactivate publishers
```

## Phase 3: Integration with Lifecycle Manager

### 3.1 Update Lifecycle Manager Node Groups
```python
# In ui/lifecycle_manager.py
self.node_groups = {
    'core': [
        'servo_interface_node',       # Hardware interface
        'ui_dashboard_node',          # UI always available
        'emergency_stop_node'         # Safety always active
    ],
    'manual': [
        'manual_control_node'         # Manual mode only
    ],
    'perception': [
        'camera_node',                # Camera only in perception mode
        'hand_pose_node',             # Hand detection only in perception
        'perception_control_node'     # Perception-to-servo mapping
    ]
}
```

### 3.2 Mode Switching Logic
```python
def switch_to_manual_mode(self):
    """Switch from any mode to manual mode"""
    # 1. Deactivate perception nodes (if active)
    for node in self.node_groups['perception']:
        self.deactivate_node(node)
    
    # 2. Configure manual nodes (if not configured)
    for node in self.node_groups['manual']:
        if self.get_node_state(node) == 'unconfigured':
            self.configure_node(node)
    
    # 3. Activate manual nodes
    for node in self.node_groups['manual']:
        self.activate_node(node)
    
    self.current_mode = 'manual'

def switch_to_perception_mode(self):
    """Switch from any mode to perception mode"""
    # 1. Deactivate manual nodes
    for node in self.node_groups['manual']:
        self.deactivate_node(node)
    
    # 2. Configure and activate perception nodes
    for node in self.node_groups['perception']:
        if self.get_node_state(node) == 'unconfigured':
            if not self.configure_node(node):
                self.get_logger().error(f"Failed to configure {node}")
                return False
        self.activate_node(node)
    
    self.current_mode = 'perception'
```

## Phase 4: Setup.py Updates

### 4.1 Entry Points Configuration
```python
# setup.py entry_points
entry_points={
    'console_scripts': [
        # Lifecycle nodes (primary)
        'servo_interface_node = control_system.servo_interface_lifecycle_node:main',
        'manual_control_node = control_system.manual_control_lifecycle_node:main',
        'perception_control_node = control_system.perception_control_lifecycle_node:main',
        'unified_control_node = control_system.unified_control_lifecycle_node:main',
        'mode_manager_node = control_system.mode_manager_lifecycle_node:main',
        
        # Legacy nodes (for transition period)
        'servo_interface_legacy = control_system.servo_interface_node:main',
        'manual_control_legacy = control_system.manual_control_node:main',
    ],
},
```

## Phase 5: Implementation Timeline

### Week 1: Foundation
- [ ] Convert `servo_interface_node.py` to lifecycle
- [ ] Test hardware connection/disconnection
- [ ] Update setup.py entry points
- [ ] Verify basic lifecycle transitions

### Week 2: Control Nodes
- [ ] Convert `manual_control_node.py` to lifecycle
- [ ] Convert `perception_control_node.py` to lifecycle
- [ ] Test mode switching with lifecycle manager
- [ ] Implement error recovery

### Week 3: Integration
- [ ] Convert `unified_control_node.py` to lifecycle
- [ ] Convert `mode_manager_node.py` to lifecycle
- [ ] Test full system integration
- [ ] Performance optimization

### Week 4: Testing & Documentation
- [ ] Comprehensive testing of all modes
- [ ] Error handling and recovery testing
- [ ] Update documentation
- [ ] Remove legacy nodes

## Benefits of Lifecycle Conversion

### 1. Resource Management
- **Hardware**: Servos only enabled when needed
- **CPU**: Perception only active in perception mode
- **Memory**: Proper cleanup of resources

### 2. Error Handling
- **Recovery**: Failed nodes can be reconfigured
- **Isolation**: One failed node doesn't crash entire system
- **Diagnostics**: Clear state information for debugging

### 3. System Robustness
- **Controlled Startup**: Dependencies managed properly
- **Graceful Shutdown**: All resources cleaned up
- **Mode Switching**: Clean transitions between operation modes

### 4. Development Benefits
- **Testing**: Individual node state testing
- **Debugging**: Clear state visibility
- **Maintenance**: Easier to modify and extend

## Testing Strategy

### Unit Testing
```python
# test_servo_interface_lifecycle.py
def test_configure_success():
    node = ServoInterfaceLifecycleNode()
    result = node.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS

def test_activate_without_configure():
    node = ServoInterfaceLifecycleNode()
    result = node.on_activate(None)
    assert result == TransitionCallbackReturn.FAILURE
```

### Integration Testing
- Test complete mode switching sequences
- Test error recovery scenarios
- Test hardware connection/disconnection
- Test resource cleanup

This conversion plan provides a structured approach to transforming the control_system package into a robust, lifecycle-managed system with proper state management and error handling capabilities.

## Analysis: Unified Control Node vs Lifecycle Manager

### Current Duplication Issue
```
unified_control_node (control_system)     vs     lifecycle_manager (ui)
├── Mode switching logic                   ├── Mode switching logic  
├── Node state management                  ├── Node state management
├── Individual node control                ├── Individual node control
└── Command interface                      └── Service interface
```

**Problem**: Both nodes essentially do the same thing - manage lifecycle states of other nodes and handle mode switching. This creates:
- Code duplication
- Potential conflicts
- Maintenance overhead
- Unclear responsibility boundaries

### Recommendation: Eliminate Unified Control Node
The `lifecycle_manager` in the UI package is the better choice because:
1. **UI Integration**: Naturally fits with dashboard and mode switching UI
2. **Centralized Control**: Single point for all lifecycle management
3. **Better Positioning**: UI package is the natural orchestrator
4. **Already Implemented**: Has working node groups and mode switching logic

## Phase 0: Centralized Action-Based Lifecycle Management System

### 0.1 System Architecture
```
┌─────────────────────────────────────────────────────────────┐
│                    Any ROS2 Node                            │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Action Client                                      │    │
│  │  - Request: target_node, transition_type           │    │
│  │  - Response: success/failure, error_message        │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────┬───────────────────────────┘
                                  │ Action Call
                                  ▼
┌─────────────────────────────────────────────────────────────┐
│                UI Package - Lifecycle Manager               │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Action Server                                      │    │
│  │  - Receive requests                                 │    │
│  │  - Execute lifecycle transitions                   │    │
│  │  - Return results                                  │    │
│  └─────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Lifecycle Service Clients                         │    │
│  │  - /node_name/change_state                         │    │
│  │  - /node_name/get_state                           │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────┬───────────────────────────┘
                                  │ Lifecycle Services
                                  ▼
┌─────────────────────────────────────────────────────────────┐
│                Target Lifecycle Nodes                       │
│  ┌─────────────────┐ ┌─────────────────┐ ┌──────────────┐   │
│  │ servo_interface │ │ manual_control  │ │ camera_node  │   │
│  │    lifecycle    │ │   lifecycle     │ │  lifecycle   │   │
│  └─────────────────┘ └─────────────────┘ └──────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 0.2 Action Interface Definition
Create a new action interface for lifecycle management:

```python
# ui/action/LifecycleManagement.action

# Goal - What transition to perform
string target_node_name          # Name of the node to control
string transition_type           # "configure", "activate", "deactivate", "cleanup", "shutdown"
bool wait_for_completion        # Whether to wait for transition to complete
float32 timeout_seconds         # Maximum time to wait (default: 5.0)
---
# Result - What happened
bool success                    # Whether the transition succeeded
string final_state             # Final state of the node
string error_message           # Error message if failed
float32 execution_time         # How long the transition took
---
# Feedback - Progress updates
string current_action          # What's currently happening
float32 progress_percentage    # 0.0 to 100.0
string status_message         # Human-readable status
```

### 0.3 Enhanced Lifecycle Manager Implementation

```python
# ui/ui/lifecycle_manager_action.py
import rclpy
from rclpy.action import ActionServer
from rclpy.lifecycle import LifecycleNode
from ui_interfaces.action import LifecycleManagement  # Custom action
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import time

class LifecycleManagerAction(LifecycleNode):
    """
    Centralized lifecycle management with action interface.
    Any node can request lifecycle transitions on any other node.
    """
    
    def __init__(self):
        super().__init__('lifecycle_manager')
        
        # Action server for lifecycle management
        self.action_server = None
        
        # Service clients for all managed nodes
        self.lifecycle_clients = {}
        self.state_clients = {}
        
        # Node registry and groups
        self.registered_nodes = set()
        self.node_groups = {
            'core': ['servo_interface_node', 'ui_dashboard_node', 'emergency_stop_node'],
            'manual': ['manual_control_node'],
            'perception': ['camera_node', 'hand_pose_node', 'perception_control_node']
        }
        
    def on_configure(self, state):
        """Configure the lifecycle manager"""
        try:
            # Create action server
            self.action_server = ActionServer(
                self,
                LifecycleManagement,
                'lifecycle_management',
                self.execute_lifecycle_action
            )
            
            # Initialize service clients for all possible nodes
            self._initialize_service_clients()
            
            self.get_logger().info("Lifecycle Manager Action Server configured")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Failed to configure: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state):
        """Activate the lifecycle manager"""
        self.get_logger().info("Lifecycle Manager Action Server activated")
        return TransitionCallbackReturn.SUCCESS
    
    async def execute_lifecycle_action(self, goal_handle):
        """Execute lifecycle transition action"""
        request = goal_handle.request
        
        # Validate request
        if not self._validate_request(request):
            goal_handle.abort()
            return LifecycleManagement.Result(
                success=False,
                error_message="Invalid request parameters"
            )
        
        # Send initial feedback
        feedback = LifecycleManagement.Feedback()
        feedback.current_action = f"Starting {request.transition_type} for {request.target_node_name}"
        feedback.progress_percentage = 0.0
        goal_handle.publish_feedback(feedback)
        
        start_time = time.time()
        
        try:
            # Execute the transition
            success = await self._execute_transition(
                request.target_node_name,
                request.transition_type,
                goal_handle,
                request.timeout_seconds
            )
            
            execution_time = time.time() - start_time
            final_state = self.get_node_state(request.target_node_name)
            
            if success:
                goal_handle.succeed()
                return LifecycleManagement.Result(
                    success=True,
                    final_state=final_state,
                    execution_time=execution_time
                )
            else:
                goal_handle.abort()
                return LifecycleManagement.Result(
                    success=False,
                    final_state=final_state,
                    error_message=f"Transition {request.transition_type} failed",
                    execution_time=execution_time
                )
                
        except Exception as e:
            goal_handle.abort()
            return LifecycleManagement.Result(
                success=False,
                error_message=f"Exception during transition: {str(e)}",
                execution_time=time.time() - start_time
            )
    
    async def _execute_transition(self, node_name, transition_type, goal_handle, timeout):
        """Execute a specific lifecycle transition"""
        
        # Map transition types to transition IDs
        transition_map = {
            'configure': Transition.TRANSITION_CONFIGURE,
            'activate': Transition.TRANSITION_ACTIVATE,
            'deactivate': Transition.TRANSITION_DEACTIVATE,
            'cleanup': Transition.TRANSITION_CLEANUP,
            'shutdown': Transition.TRANSITION_SHUTDOWN
        }
        
        if transition_type not in transition_map:
            return False
        
        transition_id = transition_map[transition_type]
        
        # Update feedback
        feedback = LifecycleManagement.Feedback()
        feedback.current_action = f"Executing {transition_type} transition"
        feedback.progress_percentage = 25.0
        goal_handle.publish_feedback(feedback)
        
        # Execute the transition
        success = self._change_node_state(node_name, transition_id, timeout)
        
        # Final feedback
        feedback.progress_percentage = 100.0
        feedback.current_action = "Transition completed"
        goal_handle.publish_feedback(feedback)
        
        return success
    
    def _validate_request(self, request):
        """Validate lifecycle management request"""
        valid_transitions = ['configure', 'activate', 'deactivate', 'cleanup', 'shutdown']
        
        if request.transition_type not in valid_transitions:
            self.get_logger().error(f"Invalid transition type: {request.transition_type}")
            return False
            
        if not request.target_node_name:
            self.get_logger().error("Empty target node name")
            return False
            
        return True
    
    # ... (rest of the existing lifecycle manager methods)
```

### 0.4 Action Client Helper Class

```python
# ui/ui/lifecycle_action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ui_interfaces.action import LifecycleManagement

class LifecycleActionClient:
    """
    Helper class for any node to request lifecycle transitions.
    Can be used as a mixin or standalone utility.
    """
    
    def __init__(self, node: Node):
        self.node = node
        self.action_client = ActionClient(
            node, 
            LifecycleManagement, 
            'lifecycle_management'
        )
    
    async def configure_node(self, node_name: str, timeout: float = 5.0):
        """Request a node to be configured"""
        return await self._send_lifecycle_request(node_name, 'configure', timeout)
    
    async def activate_node(self, node_name: str, timeout: float = 5.0):
        """Request a node to be activated"""
        return await self._send_lifecycle_request(node_name, 'activate', timeout)
    
    async def deactivate_node(self, node_name: str, timeout: float = 5.0):
        """Request a node to be deactivated"""
        return await self._send_lifecycle_request(node_name, 'deactivate', timeout)
    
    async def cleanup_node(self, node_name: str, timeout: float = 5.0):
        """Request a node to be cleaned up"""
        return await self._send_lifecycle_request(node_name, 'cleanup', timeout)
    
    async def shutdown_node(self, node_name: str, timeout: float = 5.0):
        """Request a node to be shutdown"""
        return await self._send_lifecycle_request(node_name, 'shutdown', timeout)
    
    async def _send_lifecycle_request(self, node_name: str, transition: str, timeout: float):
        """Send lifecycle management request"""
        
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("Lifecycle management action server not available")
            return False
        
        # Create goal
        goal = LifecycleManagement.Goal()
        goal.target_node_name = node_name
        goal.transition_type = transition
        goal.wait_for_completion = True
        goal.timeout_seconds = timeout
        
        # Send goal
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"Goal rejected for {transition} on {node_name}")
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        
        result = result_future.result().result
        if result.success:
            self.node.get_logger().info(
                f"Successfully {transition}ed {node_name} to state {result.final_state}"
            )
        else:
            self.node.get_logger().error(
                f"Failed to {transition} {node_name}: {result.error_message}"
            )
        
        return result.success
```

### 0.5 Usage Examples

#### From Manual Control Node
```python
# control_system/manual_control_lifecycle_node.py
class ManualControlLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('manual_control_node')
        # Initialize lifecycle action client
        self.lifecycle_client = LifecycleActionClient(self)
    
    async def on_activate(self, state):
        """When manual control activates, ensure servo interface is ready"""
        
        # Ensure servo interface is configured and activated
        if not await self.lifecycle_client.configure_node('servo_interface_node'):
            return TransitionCallbackReturn.FAILURE
            
        if not await self.lifecycle_client.activate_node('servo_interface_node'):
            return TransitionCallbackReturn.FAILURE
        
        # Now proceed with own activation
        self.servo_pub.on_activate(state)
        return TransitionCallbackReturn.SUCCESS
    
    async def on_deactivate(self, state):
        """When deactivating, optionally deactivate servo interface"""
        
        # Deactivate own resources first
        self.servo_pub.on_deactivate(state)
        
        # Optionally deactivate servo interface if no other nodes need it
        await self.lifecycle_client.deactivate_node('servo_interface_node')
        
        return TransitionCallbackReturn.SUCCESS
```

#### From UI Dashboard
```python
# ui/ui_dashboard_lifecycle_node.py
class UIDashboardLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('ui_dashboard_node')
        self.lifecycle_client = LifecycleActionClient(self)
    
    def handle_mode_switch_button(self, new_mode):
        """Handle mode switching via UI button"""
        
        if new_mode == 'manual':
            # Deactivate perception nodes
            asyncio.create_task(self.lifecycle_client.deactivate_node('camera_node'))
            asyncio.create_task(self.lifecycle_client.deactivate_node('hand_pose_node'))
            
            # Activate manual nodes
            asyncio.create_task(self.lifecycle_client.activate_node('manual_control_node'))
            
        elif new_mode == 'perception':
            # Deactivate manual nodes
            asyncio.create_task(self.lifecycle_client.deactivate_node('manual_control_node'))
            
            # Activate perception nodes
            asyncio.create_task(self.lifecycle_client.configure_node('camera_node'))
            asyncio.create_task(self.lifecycle_client.activate_node('camera_node'))
```

### 0.6 Implementation Plan

#### Week 1: Action Interface Setup
- [ ] Create `ui_interfaces` package for custom actions
- [ ] Define `LifecycleManagement.action` interface
- [ ] Create `LifecycleActionClient` helper class
- [ ] Test basic action communication

#### Week 2: Enhanced Lifecycle Manager
- [ ] Convert existing lifecycle_manager to action-based system
- [ ] Implement action server with progress feedback
- [ ] Add comprehensive error handling and validation
- [ ] Test with existing servo_interface_lifecycle_node

#### Week 3: Integration and Testing
- [ ] Update control_system nodes to use action client
- [ ] Remove unified_control_node (eliminate duplication)
- [ ] Test complex scenarios (mode switching, error recovery)
- [ ] Performance optimization

#### Week 4: Documentation and Cleanup
- [ ] Update documentation with action-based examples
- [ ] Create usage guides for developers
- [ ] Clean up deprecated code
- [ ] Final integration testing

This action-based approach provides:
1. **Single Responsibility**: Only lifecycle_manager handles lifecycle transitions
2. **Universal Access**: Any node can control any other node's lifecycle
3. **Async Support**: Non-blocking lifecycle transitions with progress feedback
4. **Centralized Logic**: All lifecycle management logic in one place
5. **Error Handling**: Comprehensive error reporting and recovery
6. **Scalability**: Easy to add new nodes and transition types
