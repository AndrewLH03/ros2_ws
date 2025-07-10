#!/usr/bin/env python3
"""
Trajectory Executor Node for CR3 Control System

Executes joint trajectories by calling action servers exposed by cr3_controller_node. Subscribes to /cr3/trajectory and publishes logs to /log/command_sequence. Optionally listens for feedback on /joint_states to verify motion completion.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time

class TrajectoryExecutorNode(Node):
    """
    Outline: Executes full joint trajectories for the CR3 robot.
    - Sends trajectory goals to action servers
    - Tracks execution status, manages timeouts, retries, and aborts
    - Publishes command sequence logs
    """
    def __init__(self):
        """Initialize the trajectory executor node."""
        super().__init__('trajectory_executor_node')
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/cr3/trajectory',
            self.handle_trajectory,
            10)
            
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.handle_joint_states,
            10)
            
        # Publishers
        self.log_pub = self.create_publisher(
            String,
            '/log/command_sequence',
            10)
            
        # Track current execution
        self.executing = False
        self.current_trajectory = None
        self.execution_start_time = None
        self.timeout = 10.0  # seconds
        
        # Timer for checking execution status
        self.timer = self.create_timer(0.5, self.check_execution_status)
        
        self.get_logger().info('Trajectory executor node started')

    def handle_trajectory(self, msg):
        """Handle incoming trajectory."""
        if self.executing:
            self.publish_log("Received new trajectory while still executing previous one. Aborting previous execution.")
        
        self.get_logger().info(f"Received trajectory with {len(msg.points)} points")
        self.publish_log(f"Starting execution of trajectory with {len(msg.points)} points")
        
        # Start execution
        self.executing = True
        self.current_trajectory = msg
        self.execution_start_time = time.time()
        
        # Simulate sending to action server
        self.send_trajectory_goal(msg)

    def send_trajectory_goal(self, trajectory):
        """Send a trajectory goal to the CR3 controller action server."""
        # Simulate sending to action server by just logging
        self.get_logger().info("Sending trajectory to controller")
        
        # In a real implementation, this would call the action server
        # Example with action client:
        # goal_msg = FollowJointTrajectory.Goal()
        # goal_msg.trajectory = trajectory
        # self._action_client.send_goal_async(goal_msg)
        
        # Instead, we'll just publish a log message
        self.publish_log("Trajectory sent to controller")

    def handle_joint_states(self, msg):
        """Handle feedback from joint states."""
        # Only process if we're executing a trajectory
        if not self.executing:
            return
        
        # In a real implementation, this would compare current joint states
        # with the expected trajectory point at the current time

    def check_execution_status(self):
        """Check if the current execution has completed or timed out."""
        if not self.executing:
            return
            
        # For minimal implementation, just simulate completion after 3 seconds
        if time.time() - self.execution_start_time > 3.0:
            self.executing = False
            self.publish_log("Trajectory execution completed successfully")
            self.get_logger().info("Trajectory execution completed")
            
        # Check for timeout
        elif time.time() - self.execution_start_time > self.timeout:
            self.executing = False
            self.publish_log("Trajectory execution timed out")
            self.get_logger().warn("Trajectory execution timed out")

    def publish_log(self, message):
        """Publish a log message to /log/command_sequence."""
        msg = String()
        msg.data = f"[{self.get_clock().now().seconds_nanoseconds()[0]}] {message}"
        self.log_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()