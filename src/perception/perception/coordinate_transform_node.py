"""
Coordinate Transform Node for CR3 Control System

Transforms pose data between coordinate frames and calculates tracking vectors.
Integrates functionality from hand tracking dashboard for vector calculation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Vector3
from std_msgs.msg import Float32
import numpy as np

class CoordinateTransformNode(Node):
    """
    Transforms pose data between coordinate frames and calculates tracking vectors.
    - Subscribes to /perception/hand_pose, /perception/body_pose
    - Calculates shoulder-to-wrist tracking vector
    - Applies coordinate transformations to robot frame
    - Publishes transformed poses and tracking vectors
    """
    def __init__(self):
        """Initialize the coordinate transform node."""
        super().__init__('coordinate_transform_node')
        
        # Subscribers
        self.hand_pose_sub = self.create_subscription(
            PoseArray,
            '/perception/hand_pose',
            self.process_hand_pose,
            10)
            
        self.body_pose_sub = self.create_subscription(
            PoseArray,
            '/perception/body_pose',
            self.process_body_pose,
            10)
        
        # Publishers
        self.hand_pose_robot_pub = self.create_publisher(
            PoseArray,
            '/perception/hand_pose_robot_frame',
            10)
            
        self.shoulder_pose_pub = self.create_publisher(
            Pose,
            '/perception/shoulder_pose',
            10)
            
        self.wrist_pose_pub = self.create_publisher(
            Pose,
            '/perception/wrist_pose',
            10)
            
        self.tracking_vector_pub = self.create_publisher(
            Vector3,
            '/perception/tracking_vector',
            10)
        
        # Store latest poses for vector calculation
        self.latest_hand_pose = None
        self.latest_body_pose = None
        
        self.get_logger().info('Coordinate transform node started')

    def process_hand_pose(self, hand_pose_array):
        """Process hand pose data and transform to robot frame."""
        if len(hand_pose_array.poses) == 0:
            return
            
        self.latest_hand_pose = hand_pose_array.poses[0]  # Assume first pose is wrist
        
        # Transform to robot frame (simplified transformation for now)
        robot_frame_poses = PoseArray()
        robot_frame_poses.header = hand_pose_array.header
        robot_frame_poses.header.frame_id = "robot_base"
        
        for pose in hand_pose_array.poses:
            transformed_pose = self.transform_to_robot_frame(pose)
            robot_frame_poses.poses.append(transformed_pose)
        
        self.hand_pose_robot_pub.publish(robot_frame_poses)
        
        # Publish wrist pose separately
        if self.latest_hand_pose:
            wrist_pose = self.transform_to_robot_frame(self.latest_hand_pose)
            self.wrist_pose_pub.publish(wrist_pose)
        
        # Calculate tracking vector if we have both poses
        self.calculate_tracking_vector()

    def process_body_pose(self, body_pose_array):
        """Process body pose data to extract shoulder position."""
        if len(body_pose_array.poses) >= 12:  # Assume shoulder is at index 11 (right shoulder)
            self.latest_body_pose = body_pose_array.poses[11]
            
            # Publish shoulder pose
            shoulder_pose = self.transform_to_robot_frame(self.latest_body_pose)
            self.shoulder_pose_pub.publish(shoulder_pose)
            
            # Calculate tracking vector if we have both poses
            self.calculate_tracking_vector()

    def transform_to_robot_frame(self, pose):
        """Transform pose from camera frame to robot frame."""
        # Simplified transformation - in practice, this would use tf2
        transformed_pose = Pose()
        
        # Apply coordinate transformation (camera to robot)
        # For now, simple scaling and axis remapping
        transformed_pose.position.x = pose.position.z * 0.5  # Depth becomes X
        transformed_pose.position.y = -pose.position.x * 0.5  # Camera X becomes -Y
        transformed_pose.position.z = pose.position.y * 0.5   # Camera Y becomes Z
        
        # Keep orientation as-is for now
        transformed_pose.orientation = pose.orientation
        
        return transformed_pose

    def calculate_tracking_vector(self):
        """Calculate tracking vector from shoulder to wrist."""
        if self.latest_body_pose is None or self.latest_hand_pose is None:
            return
        
        # Calculate vector in camera coordinates
        shoulder_pos = np.array([
            self.latest_body_pose.position.x,
            self.latest_body_pose.position.y,
            self.latest_body_pose.position.z
        ])
        
        wrist_pos = np.array([
            self.latest_hand_pose.position.x,
            self.latest_hand_pose.position.y,
            self.latest_hand_pose.position.z
        ])
        
        # Calculate vector from shoulder to wrist
        vector = wrist_pos - shoulder_pos
        
        # Transform vector to robot frame
        vector_robot = Vector3()
        vector_robot.x = float(vector[2] * 0.5)   # Depth becomes X
        vector_robot.y = float(-vector[0] * 0.5)  # Camera X becomes -Y
        vector_robot.z = float(vector[1] * 0.5)   # Camera Y becomes Z
        
        self.tracking_vector_pub.publish(vector_robot)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()