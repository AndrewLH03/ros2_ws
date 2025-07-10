#!/usr/bin/env python3
"""
        # OpenCV and MediaPipe setup
        self.cv_bridge = CvBridge()
        self.mp_pose = mp.solutions.pose
        
        # Initialize MediaPipe pose model - optimized for real-time tracking
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,  # Use medium complexity for balance
            smooth_landmarks=True,
            min_detection_confidence=0.5,  # Lower for faster detection
            min_tracking_confidence=0.7    # Higher for smoother tracking
        )or CR3 Control System

Enhanced with MediaPipe pose detection. Detects and publishes human body pose information.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
import cv2
import numpy as np
from cv_bridge import CvBridge
import mediapipe as mp

class BodyPoseNode(Node):
    """
    Enhanced body pose detection using MediaPipe.
    - Subscribes to camera/image topics
    - Runs MediaPipe pose estimation
    - Publishes body pose data with shoulder extraction
    """
    def __init__(self):
        """Initialize the enhanced body pose node."""
        super().__init__('body_pose_node')
        
        # Performance optimization: frame skipping
        self.frame_skip_count = 0
        self.frame_skip_interval = 3  # Process every 4th frame (7.5 FPS for body pose)
        
        # OpenCV and MediaPipe setup
        self.cv_bridge = CvBridge()
        self.mp_pose = mp.solutions.pose
        
        # Initialize MediaPipe pose model - optimized for performance
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=0,  # Use simpler model for better performance
            smooth_landmarks=True,
            min_detection_confidence=0.7,  # Higher threshold for better performance
            min_tracking_confidence=0.5
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            1)  # Process only latest image for minimal latency
            
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/perception/body_pose',
            10)
        
        self.get_logger().info('Enhanced body pose node started with MediaPipe')

    def process_image(self, image_msg):
        """Process incoming image and extract body pose using MediaPipe."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Convert BGR to RGB for MediaPipe (no resizing for better accuracy)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = self.pose.process(rgb_image)
            
            # Extract body poses and publish
            self.extract_and_publish_body_poses(results)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def extract_and_publish_body_poses(self, results):
        """Extract body landmarks and publish as poses."""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_frame"
        
        if results.pose_landmarks:
            # Extract all pose landmarks in order
            landmarks = [
                self.mp_pose.PoseLandmark.NOSE,
                self.mp_pose.PoseLandmark.LEFT_EYE_INNER,
                self.mp_pose.PoseLandmark.LEFT_EYE,
                self.mp_pose.PoseLandmark.LEFT_EYE_OUTER,
                self.mp_pose.PoseLandmark.RIGHT_EYE_INNER,
                self.mp_pose.PoseLandmark.RIGHT_EYE,
                self.mp_pose.PoseLandmark.RIGHT_EYE_OUTER,
                self.mp_pose.PoseLandmark.LEFT_EAR,
                self.mp_pose.PoseLandmark.RIGHT_EAR,
                self.mp_pose.PoseLandmark.MOUTH_LEFT,
                self.mp_pose.PoseLandmark.MOUTH_RIGHT,
                self.mp_pose.PoseLandmark.LEFT_SHOULDER,   # Index 11
                self.mp_pose.PoseLandmark.RIGHT_SHOULDER,  # Index 12
                self.mp_pose.PoseLandmark.LEFT_ELBOW,
                self.mp_pose.PoseLandmark.RIGHT_ELBOW,
                self.mp_pose.PoseLandmark.LEFT_WRIST,
                self.mp_pose.PoseLandmark.RIGHT_WRIST,     # Index 16
                self.mp_pose.PoseLandmark.LEFT_PINKY,
                self.mp_pose.PoseLandmark.RIGHT_PINKY,
                self.mp_pose.PoseLandmark.LEFT_INDEX,
                self.mp_pose.PoseLandmark.RIGHT_INDEX,
                self.mp_pose.PoseLandmark.LEFT_THUMB,
                self.mp_pose.PoseLandmark.RIGHT_THUMB,
                self.mp_pose.PoseLandmark.LEFT_HIP,
                self.mp_pose.PoseLandmark.RIGHT_HIP,
                self.mp_pose.PoseLandmark.LEFT_KNEE,
                self.mp_pose.PoseLandmark.RIGHT_KNEE,
                self.mp_pose.PoseLandmark.LEFT_ANKLE,
                self.mp_pose.PoseLandmark.RIGHT_ANKLE,
                self.mp_pose.PoseLandmark.LEFT_HEEL,
                self.mp_pose.PoseLandmark.RIGHT_HEEL,
                self.mp_pose.PoseLandmark.LEFT_FOOT_INDEX,
                self.mp_pose.PoseLandmark.RIGHT_FOOT_INDEX
            ]
            
            # Convert each landmark to a Pose message
            for landmark_idx in landmarks:
                landmark = results.pose_landmarks.landmark[landmark_idx]
                
                body_pose = Pose()
                body_pose.position.x = landmark.x
                body_pose.position.y = landmark.y
                body_pose.position.z = landmark.z
                body_pose.orientation.w = 1.0  # Default orientation
                
                pose_array.poses.append(body_pose)
            
            self.get_logger().debug(f'Body pose detected with {len(pose_array.poses)} landmarks')
        else:
            self.get_logger().debug('No body pose detected')
        
        # Publish pose array (even if empty)
        self.pose_pub.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = BodyPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

