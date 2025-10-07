#!/usr/bin/env python3
"""
Feature Detection Node - Phase 4 Coding Exercise

This node demonstrates ORB feature detection on live D455 camera feed.
Compare this to what RTAB-Map does internally!

Learning objectives:
1. Detect ORB features (corners) in images
2. Visualize keypoints and descriptors
3. Understand what makes a good visual landmark
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class FeatureDetectorNode(Node):
    def __init__(self):
        super().__init__('feature_detector_node')
        
        # Parameters
        self.declare_parameter('num_features', 400)
        self.declare_parameter('min_feature_distance', 7)
        self.declare_parameter('show_visualization', True)
        
        num_features = self.get_parameter('num_features').value
        
        # Create ORB detector
        self.orb = cv2.ORB_create(nfeatures=num_features)
        
        # CV Bridge for ROS<->OpenCV conversion
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for visualization
        self.viz_pub = self.create_publisher(
            Image,
            '~/feature_visualization',
            10
        )
        
        self.get_logger().info('Feature Detector Node started!')
        self.get_logger().info(f'Detecting up to {num_features} ORB features per frame')
    
    def image_callback(self, msg):
        """
        Process incoming camera image and detect features.
        
        TODO: This is your coding exercise from NEXT_SESSION_PLAN.md!
        Complete the feature detection pipeline below.
        """
        # Convert ROS Image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Convert to grayscale (ORB works on grayscale)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # ========================================
        # TODO: YOUR CODE HERE (Part 1)
        # ========================================
        # Detect keypoints and compute descriptors using self.orb
        # 
        # Hint: keypoints, descriptors = self.orb.???
        #
        keypoints, descriptors = self.orb.detectAndCompute(gray, mask=None)
        
        # ========================================
        # Analysis (observe these values!)
        # ========================================
        num_keypoints = len(keypoints)
        
        if num_keypoints > 0:
            # Analyze keypoint properties
            responses = [kp.response for kp in keypoints]
            avg_response = np.mean(responses)
            max_response = np.max(responses)
            
            # Descriptor analysis (if computed)
            if descriptors is not None:
                descriptor_info = f'Descriptor size: {descriptors.shape}'
            else:
                descriptor_info = 'No descriptors computed'
            
            # Log statistics (throttled to avoid spam)
            if num_keypoints < 50:
                self.get_logger().warn(
                    f'Low feature count: {num_keypoints} features detected'
                )
        
        # ========================================
        # Visualization
        # ========================================
        if self.get_parameter('show_visualization').value:
            # Draw keypoints on image
            viz_image = cv2.drawKeypoints(
                cv_image,
                keypoints,
                None,
                color=(0, 255, 0),  # Green keypoints
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS  # Shows orientation
            )
            
            # Add text overlay
            cv2.putText(
                viz_image,
                f'Features: {num_keypoints}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2
            )
            
            if num_keypoints > 0:
                cv2.putText(
                    viz_image,
                    f'Avg Response: {avg_response:.2f}',
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
            
            # Publish visualization
            try:
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                self.viz_pub.publish(viz_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish viz: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FeatureDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
