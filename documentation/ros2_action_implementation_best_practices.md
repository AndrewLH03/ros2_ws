# ROS2 Action Implementation Best Practices

## Overview

This document outlines best practices for implementing ROS2 actions based on official ROS2 documentation and real-world usage patterns. It serves as a guide for developing robust, maintainable action servers and clients.

## What are ROS2 Actions?

Actions are a communication pattern in ROS2 designed for **long-running tasks** that require:
- **Goal-oriented execution**: Request to perform a specific task
- **Continuous feedback**: Regular updates during execution
- **Preemptability**: Ability to cancel/abort goals mid-execution
- **Result reporting**: Final outcome when the task completes

Actions are built on top of topics and services, providing a higher-level abstraction for complex asynchronous operations.

## Action Components

### 1. Action Definition (.action file)

```
# Goal - Request sent from client to server
int32 order
---
# Result - Final response sent from server to client
int32[] sequence
---
# Feedback - Periodic updates sent from server to client
int32[] partial_sequence
```

**Best Practices:**
- Use clear, descriptive field names
- Keep goal definitions simple and focused
- Provide meaningful feedback for long-running operations
- Include comprehensive result information
- Document field units and expected ranges in comments

### 2. Action Server Implementation

#### Core Structure
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_package.action import YourAction

class YourActionServer(Node):
    def __init__(self):
        super().__init__('your_action_server')
        self._action_server = ActionServer(
            self,
            YourAction,
            'your_action_name',
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle):
        # Implementation here
        pass
```

#### Best Practices for Action Servers

1. **Goal Validation**
   ```python
   def execute_callback(self, goal_handle):
       # Validate goal parameters early
       if goal_handle.request.order < 0:
           goal_handle.abort()
           result = YourAction.Result()
           result.error_message = "Order must be non-negative"
           return result
   ```

2. **Proper State Management**
   ```python
   def execute_callback(self, goal_handle):
       self.get_logger().info('Executing goal...')
       
       # Check for cancellation periodically
       if goal_handle.is_cancel_requested:
           goal_handle.canceled()
           result = YourAction.Result()
           return result
       
       # Mark successful completion
       goal_handle.succeed()
       return result
   ```

3. **Regular Feedback Publishing**
   ```python
   def execute_callback(self, goal_handle):
       feedback_msg = YourAction.Feedback()
       
       for i in range(goal_handle.request.order):
           # Check for cancellation
           if goal_handle.is_cancel_requested:
               goal_handle.canceled()
               return YourAction.Result()
           
           # Update and publish feedback
           feedback_msg.partial_sequence.append(i)
           goal_handle.publish_feedback(feedback_msg)
           
           # Simulate work with appropriate timing
           time.sleep(0.1)
   ```

4. **Error Handling and Logging**
   ```python
   def execute_callback(self, goal_handle):
       try:
           # Main execution logic
           result = self._perform_task(goal_handle)
           goal_handle.succeed()
           return result
       except Exception as e:
           self.get_logger().error(f'Action failed: {str(e)}')
           goal_handle.abort()
           result = YourAction.Result()
           result.error_message = str(e)
           return result
   ```

### 3. Action Client Implementation

#### Core Structure
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from your_package.action import YourAction

class YourActionClient(Node):
    def __init__(self):
        super().__init__('your_action_client')
        self._action_client = ActionClient(self, YourAction, 'your_action_name')
    
    def send_goal(self, order):
        goal_msg = YourAction.Goal()
        goal_msg.order = order
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
```

#### Best Practices for Action Clients

1. **Robust Goal Handling**
   ```python
   def goal_response_callback(self, future):
       goal_handle = future.result()
       if not goal_handle.accepted:
           self.get_logger().info('Goal rejected')
           return
       
       self.get_logger().info('Goal accepted')
       self._get_result_future = goal_handle.get_result_async()
       self._get_result_future.add_done_callback(self.get_result_callback)
   ```

2. **Comprehensive Result Processing**
   ```python
   def get_result_callback(self, future):
       result = future.result().result
       status = future.result().status
       
       if status == GoalStatus.STATUS_SUCCEEDED:
           self.get_logger().info(f'Goal succeeded: {result.sequence}')
       elif status == GoalStatus.STATUS_ABORTED:
           self.get_logger().warn(f'Goal aborted: {result.error_message}')
       elif status == GoalStatus.STATUS_CANCELED:
           self.get_logger().info('Goal was canceled')
   ```

3. **Feedback Processing**
   ```python
   def feedback_callback(self, feedback_msg):
       feedback = feedback_msg.feedback
       self.get_logger().info(f'Progress: {feedback.partial_sequence}')
   ```

4. **Server Availability Checking**
   ```python
   def send_goal(self, order):
       if not self._action_client.wait_for_server(timeout_sec=5.0):
           self.get_logger().error('Action server not available')
           return None
       
       # Proceed with goal sending
   ```

## Package Configuration Best Practices

### 1. CMakeLists.txt (for action interfaces)
```cmake
cmake_minimum_required(VERSION 3.8)
project(your_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/YourAction.action"
  DEPENDENCIES action_msgs
)

ament_package()
```

### 2. package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>your_package</name>
  <version>0.0.0</version>
  <description>Your package description</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  
  <!-- Action dependencies -->
  <depend>action_msgs</depend>
  <depend>rclpy</depend>
  
  <!-- Interface generation -->
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 3. setup.py (for Python executables)
```python
from setuptools import setup

package_name = 'your_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Your package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'your_action_server = your_package.your_action_server:main',
            'your_action_client = your_package.your_action_client:main',
        ],
    },
)
```

## Common Patterns and Anti-Patterns

### ✅ Do's

1. **Always validate goal parameters** before starting execution
2. **Check for cancellation requests** regularly during long operations
3. **Provide meaningful feedback** at appropriate intervals
4. **Use proper state transitions** (succeed, abort, cancel)
5. **Handle exceptions gracefully** with appropriate logging
6. **Wait for server availability** before sending goals
7. **Use descriptive action and field names**
8. **Document expected behavior** in action definitions

### ❌ Don'ts

1. **Don't block indefinitely** without checking cancellation
2. **Don't ignore goal validation** - validate early and fail fast
3. **Don't publish feedback too frequently** - respect system resources
4. **Don't forget to set goal state** - always call succeed(), abort(), or cancel()
5. **Don't use actions for simple request-response** - use services instead
6. **Don't make feedback messages too large** - keep them lightweight
7. **Don't ignore error conditions** - always handle and report errors

## Testing Best Practices

### 1. Unit Testing Action Servers
```python
import unittest
from rclpy.action import ActionServer
from your_package.your_action_server import YourActionServer

class TestYourActionServer(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = YourActionServer()
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_goal_validation(self):
        # Test goal validation logic
        pass
    
    def test_successful_execution(self):
        # Test successful goal execution
        pass
```

### 2. Integration Testing
- Test action server/client communication
- Verify feedback frequency and content
- Test cancellation scenarios
- Validate error handling paths

## Performance Considerations

1. **Feedback Frequency**: Balance between informativeness and system load
2. **Goal Queue Management**: Consider implementing goal queuing for busy servers
3. **Resource Cleanup**: Ensure proper cleanup of resources on cancellation/abort
4. **Timeout Handling**: Implement appropriate timeouts for long-running operations

## Security Considerations

1. **Input Validation**: Always validate goal parameters for security
2. **Resource Limits**: Implement limits to prevent resource exhaustion
3. **Access Control**: Consider authentication/authorization for sensitive actions
4. **Logging**: Log security-relevant events appropriately

## Lifecycle Integration

When integrating actions with lifecycle nodes:

```python
from rclpy_lifecycle import LifecycleNode
from lifecycle_msgs.msg import Transition

class LifecycleActionServer(LifecycleNode):
    def on_configure(self, state):
        self._action_server = ActionServer(
            self,
            YourAction,
            'your_action_name',
            self.execute_callback
        )
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        self._action_server.destroy()
        return TransitionCallbackReturn.SUCCESS
```

## Summary

ROS2 actions provide a powerful pattern for managing long-running, interruptible tasks. Following these best practices ensures:

- **Reliability**: Robust error handling and state management
- **Usability**: Clear interfaces and meaningful feedback
- **Performance**: Efficient resource usage and appropriate timing
- **Maintainability**: Clean, well-structured code
- **Testability**: Comprehensive testing strategies

For complex systems like robotics applications, actions should be the preferred choice for operations like navigation, manipulation, and other goal-oriented tasks that require progress monitoring and cancellation capabilities.

## References

- [ROS2 Official Action Tutorials](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html)
- [ROS2 Action Design Document](https://design.ros2.org/articles/actions.html)
- [ROS2 Examples Repository](https://github.com/ros2/examples)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/YourAction.action"
  # Add more actions if needed:
  # "action/AnotherAction.action"
  DEPENDENCIES action_msgs  # Add other dependencies here
  # std_msgs geometry_msgs
)

# Install Python package (hybrid approach)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### 2. package.xml (Hybrid Package Configuration)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <n>package_name</n>
  <version>0.1.0</version>
  <description>Package description with action interfaces</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <!-- Hybrid build dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Action interface generation -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <!-- Python dependencies -->
  <depend>rclpy</depend>
  <depend>rclpy_action</depend>
  
  <!-- Action and message dependencies -->
  <depend>action_msgs</depend>
  <depend>std_msgs</depend>
  <!-- Add other message packages as needed -->

  <!-- Action interface package membership -->
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>  <!-- CMake build for interface generation -->
  </export>
</package>
```

### 3. setup.py (Python Package Configuration)

```python
from setuptools import setup, find_packages

package_name = 'package_name'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'rclpy_action',
        'action_msgs',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package description with action interfaces',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server_node = package_name.action_server_node:main',
            'action_client_node = package_name.action_client_node:main',
            # Add other nodes as needed
        ],
    },
)
```

### 4. setup.cfg (Critical for Executable Placement)

```ini
[develop]
script_dir = src/package_name
[install]
install_scripts = $base/lib/package_name
```

---

## 🔧 Action Interface Definition Best Practices

### Action File Template (action/YourAction.action)

```plaintext
# Goal - What the client wants to achieve
string target_name                    # Target identifier
string operation_type                 # Type of operation to perform
bool wait_for_completion             # Whether to wait for completion
float32 timeout_seconds              # Maximum execution time
---
# Result - What happened after completion
bool success                         # Whether the operation succeeded
string final_state                   # Final state after operation
string error_message                 # Error details if failed
float32 execution_time              # Actual execution time
---
# Feedback - Progress updates during execution
string current_action               # Current sub-action being performed
float32 progress_percentage         # Progress from 0.0 to 100.0
string status_message              # Human-readable status update
```

---

## 🐍 Python Implementation Templates

### Action Server Template

```python
#!/usr/bin/env python3
"""
Action Server Node Template
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

# Import your action - will be available after building
try:
    from package_name.action import YourAction
except ImportError:
    # Fallback for development
    print("Action interface not available - build the package first")
    class YourAction:
        class Goal:
            pass
        class Result:
            pass
        class Feedback:
            pass

class YourActionServer(Node):
    """Action server implementation."""
    
    def __init__(self):
        super().__init__('your_action_server')
        
        # Use reentrant callback group for concurrent action handling
        self._action_group = ReentrantCallbackGroup()
        
        # Create action server
        self._action_server = ActionServer(
            self,
            YourAction,
            'your_action_name',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group
        )
        
        self.get_logger().info('Action server started')
    
    def goal_callback(self, goal_request):
        """Accept or reject incoming goals."""
        self.get_logger().info(f'Received goal request: {goal_request}')
        # Add validation logic here
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute the action."""
        self.get_logger().info('Executing goal...')
        goal = goal_handle.request
        
        # Initialize feedback and result
        feedback_msg = YourAction.Feedback()
        result = YourAction.Result()
        
        try:
            # Simulate work with progress updates
            for i in range(10):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.error_message = "Action was canceled"
                    return result
                
                # Update progress
                feedback_msg.progress_percentage = float(i * 10)
                feedback_msg.current_action = f"Step {i+1}/10"
                feedback_msg.status_message = f"Processing step {i+1}"
                goal_handle.publish_feedback(feedback_msg)
                
                # Simulate work
                await asyncio.sleep(0.5)
            
            # Success
            goal_handle.succeed()
            result.success = True
            result.final_state = "completed"
            result.execution_time = 5.0
            
        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.error_message = str(e)
        
        return result

def main(args=None):
    rclpy.init(args=args)
    
    action_server = YourActionServer()
    
    # Use MultiThreadedExecutor for concurrent action handling
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Template

```python
#!/usr/bin/env python3
"""
Action Client Node Template
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import your action
try:
    from package_name.action import YourAction
except ImportError:
    print("Action interface not available - build the package first")
    class YourAction:
        class Goal:
            pass

class YourActionClient(Node):
    """Action client implementation."""
    
    def __init__(self):
        super().__init__('your_action_client')
        self._action_client = ActionClient(self, YourAction, 'your_action_name')
    
    def send_goal(self, target_name, operation_type, timeout=5.0):
        """Send a goal to the action server."""
        goal_msg = YourAction.Goal()
        goal_msg.target_name = target_name
        goal_msg.operation_type = operation_type
        goal_msg.wait_for_completion = True
        goal_msg.timeout_seconds = timeout
        
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return None
        
        self.get_logger().info(f'Sending goal: {goal_msg}')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        return send_goal_future
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {feedback.progress_percentage}% - {feedback.status_message}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    action_client = YourActionClient()
    
    # Example usage
    future = action_client.send_goal('test_target', 'configure')
    
    rclpy.spin_until_future_complete(action_client, future)
    
    if future.result() is not None:
        goal_handle = future.result()
        if goal_handle.accepted:
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(action_client, get_result_future)
            result = get_result_future.result().result
            action_client.get_logger().info(f'Result: {result}')
    
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🏗️ Build Process Best Practices

### Build Sequence

```bash
# 1. Clean previous builds (recommended)
rm -rf build/ install/ log/

# 2. Source ROS2 environment
source /opt/ros/jazzy/setup.bash  # or your ROS2 distribution

# 3. Build the package (DO NOT use --symlink-install with hybrid packages)
colcon build --packages-select package_name

# 4. Source the workspace
source install/setup.bash

# 5. Verify action interface generation
ros2 interface list | grep package_name
ros2 interface show package_name/action/YourAction
```

### Testing Action Implementation

```bash
# Test action server
ros2 run package_name action_server_node

# In another terminal, test action client
ros2 run package_name action_client_node

# Or use command line action client
ros2 action send_goal /your_action_name package_name/action/YourAction "{target_name: 'test', operation_type: 'configure'}"

# Monitor action feedback
ros2 action send_goal /your_action_name package_name/action/YourAction "{target_name: 'test', operation_type: 'configure'}" --feedback
```

---

## 🚨 Common Pitfalls and Solutions

### Error 1: "ModuleNotFoundError: No module named 'package_name.action'"
**Cause:** Action interfaces not generated or package not built properly
**Solution:**
- Ensure CMakeLists.txt has `rosidl_generate_interfaces()`
- Build without `--symlink-install`
- Check `install/package_name/lib/python3.x/site-packages/package_name/`

### Error 2: "package 'package_name' not found"
**Cause:** Missing or incorrect package.xml dependencies
**Solution:**
- Add `<member_of_group>rosidl_interface_packages</member_of_group>`
- Include `rosidl_default_generators` and `rosidl_default_runtime`

### Error 3: Action server/client connection fails
**Cause:** Action name mismatch or server not started
**Solution:**
- Verify action names match exactly
- Use `ros2 action list` to check available actions
- Check server initialization order

### Error 4: Executables not found after build
**Cause:** Missing setup.cfg file
**Solution:**
- Add setup.cfg with install_scripts directive
- Rebuild package completely

---

## 📊 CR3 System Manager Implementation Analysis

### Current Implementation Status

Our `system_manager` package successfully implements the hybrid approach:

```
✅ CMakeLists.txt - Generates LifecycleManagement action interface
✅ package.xml - Correct hybrid dependencies and interface group membership
✅ setup.py - Python package with proper entry points
✅ setup.cfg - Correct executable placement
✅ action/LifecycleManagement.action - Well-defined action interface
✅ Python modules - Action server and lifecycle management implementations
```

### Verification Commands

```bash
# Verify action interface generation
ros2 interface show system_manager/action/LifecycleManagement

# Test action server
ros2 run system_manager lifecycle_action_server

# Test action availability
ros2 action list | grep lifecycle

# Send test action goal
ros2 action send_goal /lifecycle_management system_manager/action/LifecycleManagement "{target_node_name: 'test_node', transition_type: 'configure'}"
```

---

## 💡 Key Best Practices Summary

### ✅ DO:
- Use hybrid approach (CMake + setup.py) for Python packages with actions
- Include `setup.cfg` file for correct executable placement
- Use `member_of_group>rosidl_interface_packages</member_of_group>` in package.xml
- Build without `--symlink-install` flag
- Use MultiThreadedExecutor for action servers
- Implement proper error handling and cancellation
- Test both programmatic and command-line interfaces

### ❌ DON'T:
- Use pure Python packaging for packages with custom actions
- Forget the `rosidl_generate_interfaces()` in CMakeLists.txt
- Mix different action naming conventions
- Ignore feedback mechanisms in long-running actions
- Use blocking operations in action execute callbacks

---

## 🚀 Advanced Features

### Concurrent Action Handling

```python
# Use ReentrantCallbackGroup for concurrent actions
self._action_group = ReentrantCallbackGroup()
self._action_server = ActionServer(
    self, YourAction, 'action_name',
    execute_callback=self.execute_callback,
    callback_group=self._action_group
)
```

### Action Composition

```python
# Chain multiple actions together
async def complex_action(self, goal_handle):
    # Call sub-action 1
    result1 = await self.call_sub_action(action1_goal)
    if not result1.success:
        return self.create_failure_result(result1.error_message)
    
    # Call sub-action 2
    result2 = await self.call_sub_action(action2_goal)
    
    return self.create_success_result()
```

### Error Recovery Patterns

```python
def execute_with_retry(self, goal_handle, max_retries=3):
    """Execute action with automatic retry on failure."""
    for attempt in range(max_retries):
        try:
            return self.execute_action(goal_handle)
        except Exception as e:
            if attempt == max_retries - 1:
                raise e
            self.get_logger().warn(f'Attempt {attempt + 1} failed, retrying...')
```

---

## 📚 References

### Official ROS2 Documentation
- [ROS2 Actions Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [rosidl Interface Generation](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html)
- [Python Package Creation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

### CR3 Workspace Documentation
- `ros2_lessons_learned_best_practices.md` - Python packaging best practices
- `control_system_lifecycle_conversion_plan.md` - Lifecycle management implementation

---

**This documentation represents validated best practices from successful implementation in a production robotics workspace with action-based lifecycle management.**

---

*Happy coding! 🤖*
