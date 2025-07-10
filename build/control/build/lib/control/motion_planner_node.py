#!/usr/bin/env python3
"""
Motion Planner Node for CR3 Control System

Generates smooth, feasible joint trajectories from desired end-effector poses. Publishes trajectory_msgs/JointTrajectory messages to /cr3/trajectory.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import numpy as np

class MotionPlannerNode(Node):
    """
    Outline: Generates joint trajectories for the CR3 robot.
    - Integrates with MoveIt2 or custom kinematics
    - Checks constraints and collisions
    - Publishes planned trajectories
    """
    def __init__(self):
        """Initialize the motion planner node."""
        super().__init__('motion_planner_node')
        
        # Subscribers
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.handle_target_pose,
            10)
            
        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/cr3/trajectory',
            10)
            
        # Joint names for CR3 robot
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Timer for publishing demo trajectory (to make topics visible)
        self.timer = self.create_timer(2.0, self.publish_demo_trajectory)
        
        self.get_logger().info('Motion planner node started')

    def handle_target_pose(self, msg):
        """Handle incoming target pose."""
        self.get_logger().info(f'Received target pose at x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}')
        
        # Generate and publish a trajectory
        trajectory = self.generate_trajectory(msg.pose)
        
        if self.check_collisions(trajectory):
            self.publish_trajectory(trajectory)
        else:
            self.get_logger().warn('Generated trajectory has collisions!')

    def generate_trajectory(self, target_pose):
        """Generate a trajectory for the given target pose."""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "base_link"
        trajectory.joint_names = self.joint_names
        
        # Simple linear trajectory with 5 points
        n_points = 5
        for i in range(n_points):
            point = JointTrajectoryPoint()
            
            # Linearly interpolated joint positions (dummy values)
            t = i / (n_points - 1)
            # Example joint positions for demonstration
            point.positions = [
                0.0,          # shoulder_pan_joint
                -0.5 * t,     # shoulder_lift_joint
                0.8 * t,      # elbow_joint
                -0.3 * t,     # wrist_1_joint
                0.0,          # wrist_2_joint
                0.0           # wrist_3_joint
            ]
            
            # Set time from start
            point.time_from_start.sec = int(2.0 * t)
            point.time_from_start.nanosec = int((2.0 * t - int(2.0 * t)) * 1e9)
            
            trajectory.points.append(point)
        
        return trajectory

    def check_collisions(self, trajectory):
        """Check if the trajectory is collision-free."""
        # Simplified implementation - always return True
        return True

    def publish_trajectory(self, trajectory):
        """Publish the planned trajectory to /cr3/trajectory."""
        self.trajectory_pub.publish(trajectory)
        
    def publish_demo_trajectory(self):
        """Publish a demo trajectory to make topic visible in rqt_graph."""
        # Create a dummy pose
        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.5
        pose.orientation.w = 1.0
        
        # Generate and publish trajectory
        trajectory = self.generate_trajectory(pose)
        self.publish_trajectory(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()