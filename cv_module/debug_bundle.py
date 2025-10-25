#!/usr/bin/env python3
"""
Quick test to debug bundle detection issues
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from dt_apriltags import Detector
import yaml

class BundleDebugger(Node):
    def __init__(self):
        super().__init__('bundle_debugger')
        
        # Load bundle config
        with open('cv_module/config/apriltag3_bundle.yaml', 'r') as f:
            config = yaml.safe_load(f)
        self.bundle_config = config['apriltag3_bundle']['ros__parameters']
        
        # Convert corners to numpy arrays
        self.tag_corners = {}
        for tag_id_str, corners in self.bundle_config['tag_corners'].items():
            tag_id = int(tag_id_str)
            self.tag_corners[tag_id] = np.array(corners, dtype=np.float32)
        
        # Initialize detector
        self.detector = Detector(families='tag16h5')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to image
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        
        self.get_logger().info("Bundle debugger ready")
    
    def image_callback(self, msg: Image):
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect tags
            detections = self.detector.detect(gray_image)
            
            # Filter to bundle tags
            bundle_detections = []
            for detection in detections:
                if detection.tag_id in self.tag_corners:
                    if detection.hamming <= 0:  # Perfect detections only
                        bundle_detections.append(detection)
            
            self.get_logger().info(f"Detected {len(detections)} total tags, {len(bundle_detections)} bundle tags")
            
            if len(bundle_detections) >= 3:
                self.get_logger().info(f"Bundle tags: {[d.tag_id for d in bundle_detections]}")
                
                # Collect corners
                object_points = []
                image_points = []
                
                for detection in bundle_detections:
                    tag_id = detection.tag_id
                    corners_3d = self.tag_corners[tag_id]
                    corners_2d = detection.corners[[3, 2, 1, 0], :]  # Reorder
                    
                    object_points.extend(corners_3d)
                    image_points.extend(corners_2d)
                
                self.get_logger().info(f"Collected {len(object_points)} object points, {len(image_points)} image points")
                
                if len(object_points) >= 12:
                    self.get_logger().info("✅ Bundle detection should work!")
                else:
                    self.get_logger().warn(f"❌ Not enough points: {len(object_points)}")
            else:
                self.get_logger().warn(f"❌ Not enough bundle tags: {len(bundle_detections)}")
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    node = BundleDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
