#!/usr/bin/env python3
"""
Ha        # Hand tracking preference (default to right hand)
        self.preferred_hand = "Right"  # "Left" or "Right"
        
        # OpenCV and MediaPipe setup
        self.cv_bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        
        # Initialize MediaPipe hands model - optimized for real-time tracking
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,  # Track both hands to choose preferred one
            min_detection_confidence=0.5,  # Lower for faster detection
            min_tracking_confidence=0.7,   # Higher for smoother tracking
            model_complexity=1  # Use medium complexity for balance
        )3 Control System

Enhanced with MediaPipe hand tracking. Detects and publishes human hand pose information.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32, String
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
    - Supports hand selection (left/right)
    """
    def __init__(self):
        """Initialize the enhanced hand pose node."""
        super().__init__('hand_pose_node')
        
        # Hand tracking preference (default to right hand)
        self.preferred_hand = "Right"  # "Left" or "Right"
        
        # Performance optimization: frame skipping
        self.frame_skip_count = 0
        self.frame_skip_interval = 2  # Process every 3rd frame (10 FPS for MediaPipe)
        
        # OpenCV and MediaPipe setup
        self.cv_bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        
        # Initialize MediaPipe hands model - optimized for performance
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,  # Track both hands to choose preferred one
            min_detection_confidence=0.7,  # Higher threshold for better performance
            min_tracking_confidence=0.5,
            model_complexity=0  # Use simpler model for better performance
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            1)  # Process only latest image for minimal latency
            
        # Subscribe to hand selection commands
        self.hand_selection_sub = self.create_subscription(
            String,
            '/ui/hand_selection',
            self.handle_hand_selection,
            5)
            
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/perception/hand_pose',
            10)
            
        self.confidence_pub = self.create_publisher(
            Float32,
            '/perception/hand_confidence',
            10)
        
        self.get_logger().info(f'Enhanced hand pose node started with MediaPipe (tracking {self.preferred_hand} hand)')

    def handle_hand_selection(self, msg):
        """Handle hand selection commands from UI."""
        hand = msg.data
        if hand in ["Left", "Right"]:
            self.preferred_hand = hand
            self.get_logger().info(f'Hand tracking preference changed to: {self.preferred_hand}')
        else:
            self.get_logger().warn(f'Invalid hand selection: {hand}')

    def process_image(self, image_msg):
        """Process incoming image and extract hand pose using MediaPipe."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Convert BGR to RGB for MediaPipe (no resizing for better accuracy)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = self.hands.process(rgb_image)
            
            # Extract hand poses and publish
            self.extract_and_publish_hand_poses(results, cv_image.shape)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def extract_and_publish_hand_poses(self, results, image_shape):
        """Extract hand landmarks and publish as poses for preferred hand."""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_frame"
        
        confidence = 0.0
        
        # Find the preferred hand among detected hands
        if results.multi_hand_landmarks and results.multi_handedness:
            preferred_hand_landmarks = None
            preferred_confidence = 0.0
            
            # Look for the preferred hand
            # Note: MediaPipe labels from camera perspective, so we need to flip
            # When user wants "Right" hand, we look for MediaPipe's "Left" label
            mediapipe_label = "Left" if self.preferred_hand == "Right" else "Right"
            
            for i, (hand_landmarks, handedness) in enumerate(zip(results.multi_hand_landmarks, results.multi_handedness)):
                hand_label = handedness.classification[0].label
                hand_confidence = handedness.classification[0].score
                
                if hand_label == mediapipe_label:
                    preferred_hand_landmarks = hand_landmarks
                    preferred_confidence = hand_confidence
                    self.get_logger().debug(f'Found {self.preferred_hand} hand (MediaPipe: {hand_label})')
                    break
            
            # Only process if we found the specifically requested hand (no fallback)
            if preferred_hand_landmarks:
                confidence = preferred_confidence
                for landmark in preferred_hand_landmarks.landmark:
                    pose = Pose()
                    pose.position.x = landmark.x
                    pose.position.y = landmark.y
                    pose.position.z = landmark.z
                    pose.orientation.w = 1.0
                    pose_array.poses.append(pose)
            else:
                self.get_logger().debug(f'Preferred hand ({self.preferred_hand}) not detected, ignoring other hands')
        
        # Publish results
        self.pose_pub.publish(pose_array)
        
        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.confidence_pub.publish(confidence_msg)
        
        # Log detection status
        if confidence > 0:
            landmark_count = len(pose_array.poses)
            self.get_logger().debug(f'{self.preferred_hand} hand detected with confidence: {confidence:.2f}, landmarks: {landmark_count}')
        else:
            self.get_logger().debug(f'No {self.preferred_hand.lower()} hand detected')

def main(args=None):
    rclpy.init(args=args)
    node = HandPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

