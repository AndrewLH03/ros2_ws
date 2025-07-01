#!/usr/bin/env python3
"""
Hand Pose Node for CR3 Control System

Enhanced with MediaPipe hand tracking. Detects and publishes human hand pose information.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32
import cv2
import numpy as np
from cv_bridge import CvBridge
import mediapipe as mp

class HandPoseNode(Node):
    """
    Enhanced hand pose detection using MediaPipe.
    - Subscribes to camera/image topics
    - Runs MediaPipe hand pose estimation
    - Publishes hand pose data with confidence
    """
    def __init__(self):
        """Initialize the enhanced hand pose node."""
        super().__init__('hand_pose_node')
        
        # OpenCV and MediaPipe setup
        self.cv_bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Initialize MediaPipe hands model
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Parameters
        self.tracked_hand_label = "Right"  # Track right hand by default
        self.mirror_camera = False
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            10)
            
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/perception/hand_pose',
            10)
            
        self.confidence_pub = self.create_publisher(
            Float32,
            '/perception/hand_confidence',
            10)
        
        self.get_logger().info('Enhanced hand pose node started with MediaPipe')

    def process_image(self, image_msg):
        """Process incoming image and extract hand pose using MediaPipe."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Convert BGR to RGB for MediaPipe
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = self.hands.process(rgb_image)
            
            # Extract hand poses and publish
            self.extract_and_publish_hand_poses(results, cv_image.shape)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def extract_and_publish_hand_poses(self, results, image_shape):
        """Extract hand landmarks and publish as poses."""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_frame"
        
        confidence = 0.0
        
        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
                # Get hand label (Left/Right)
                label = hand_info.classification[0].label
                detection_confidence = hand_info.classification[0].score
                
                # Handle mirrored camera
                if self.mirror_camera:
                    label = "Left" if label == "Right" else "Right"
                
                # Only process the tracked hand
                if label == self.tracked_hand_label:
                    confidence = detection_confidence
                    
                    # Extract ALL 21 hand landmarks (just like MediaPipe reference)
                    # MediaPipe hands provides landmarks 0-20 for each hand
                    for i, landmark in enumerate(hand_landmarks.landmark):
                        pose = Pose()
                        pose.position.x = landmark.x
                        pose.position.y = landmark.y
                        pose.position.z = landmark.z
                        pose.orientation.w = 1.0  # Default orientation
                        pose_array.poses.append(pose)
                    
                    break  # Only process first matching hand
        
        # Publish pose array (even if empty)
        self.pose_pub.publish(pose_array)
        
        # Publish confidence
        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.confidence_pub.publish(confidence_msg)
        
        # Log detection status
        if confidence > 0:
            landmark_count = len(pose_array.poses)
            self.get_logger().debug(f'Hand detected with confidence: {confidence:.2f}, landmarks: {landmark_count}')
        else:
            self.get_logger().debug('No hand detected')

def main(args=None):
    rclpy.init(args=args)
    node = HandPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

