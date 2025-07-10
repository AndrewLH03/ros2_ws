#!/usr/bin/env python3
"""
Test script for enhanced hand pose node with finger extraction
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray
import time

class FingerDataTester(Node):
    def __init__(self):
        super().__init__('finger_data_tester')
        
        # Subscribe to new finger data topics
        self.curl_sub = self.create_subscription(
            Float32MultiArray, '/perception/finger_curl_ratios', 
            self.curl_callback, 10)
        
        self.angle_sub = self.create_subscription(
            Float32MultiArray, '/perception/finger_angles',
            self.angle_callback, 10)
        
        self.servo_sub = self.create_subscription(
            Float32MultiArray, '/servo/finger_commands',
            self.servo_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseArray, '/perception/hand_pose',
            self.pose_callback, 10)
        
        self.last_update = time.time()
        self.get_logger().info('Finger data tester started - monitoring enhanced hand pose node')
    
    def curl_callback(self, msg):
        fingers = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        curl_str = ', '.join([f'{fingers[i]}: {msg.data[i]:.2f}' for i in range(len(msg.data))])
        self.get_logger().info(f'Finger Curls - {curl_str}')
    
    def angle_callback(self, msg):
        fingers = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        angle_str = ', '.join([f'{fingers[i]}: {msg.data[i]:.2f}' for i in range(len(msg.data))])
        self.get_logger().info(f'Finger Angles - {angle_str}')
    
    def servo_callback(self, msg):
        fingers = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        servo_str = ', '.join([f'{fingers[i]}: {int(msg.data[i])}' for i in range(len(msg.data))])
        self.get_logger().info(f'Servo Commands - {servo_str}')
    
    def pose_callback(self, msg):
        landmark_count = len(msg.poses)
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_update) if current_time - self.last_update > 0 else 0
        self.last_update = current_time
        self.get_logger().info(f'Hand Pose - {landmark_count} landmarks, {fps:.1f} FPS')

def main(args=None):
    rclpy.init(args=args)
    node = FingerDataTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
