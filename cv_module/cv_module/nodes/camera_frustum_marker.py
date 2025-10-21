#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from typing import Optional

class CameraFrustumMarker(Node):
    def __init__(self):
        super().__init__('camera_frustum_marker')
        
        # Parameters
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('near', 0.01)  # 1cm
        self.declare_parameter('far', 0.2)    # 20cm
        
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.near = self.get_parameter('near').get_parameter_value().double_value
        self.far = self.get_parameter('far').get_parameter_value().double_value
        
        # Camera info
        self.camera_info: Optional[CameraInfo] = None
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/cv_module/camera_frustum', 10)
        
        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 
            self.camera_info_topic, 
            self.camera_info_callback, 
            10
        )
        
        self.get_logger().info("Camera frustum marker ready")
    
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.publish_frustum()
    
    def publish_frustum(self):
        if self.camera_info is None:
            return
        
        # Extract camera parameters
        fx = self.camera_info.k[0]  # Focal length x
        fy = self.camera_info.k[4]  # Focal length y
        width = self.camera_info.width
        height = self.camera_info.height
        
        # Calculate field of view angles
        fov_x = 2 * np.arctan(width / (2 * fx))
        fov_y = 2 * np.arctan(height / (2 * fy))
        
        # Calculate frustum edges including lines from camera origin
        points = self.calculate_frustum_edges(fov_x, fov_y, self.near, self.far)
        
        # Create frustum marker
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_frustum"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # Set scale and color
        marker.scale.x = 0.003  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Set lifetime
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        marker.points = points
        
        self.marker_pub.publish(marker)
    
    def calculate_frustum_corners(self, fov_x: float, fov_y: float, distance: float):
        """Calculate frustum corners at given distance"""
        # Calculate half-width and half-height at the given distance
        half_width = distance * np.tan(fov_x / 2)
        half_height = distance * np.tan(fov_y / 2)
        
        # Define corners in camera frame (Z is forward, X is right, Y is down)
        corners = [
            # Top-left
            [-half_width, -half_height, distance],
            # Top-right
            [half_width, -half_height, distance],
            # Bottom-right
            [half_width, half_height, distance],
            # Bottom-left
            [-half_width, half_height, distance]
        ]
        
        # Convert to Point messages
        points = []
        for corner in corners:
            point = Point()
            point.x = float(corner[0])
            point.y = float(corner[1])
            point.z = float(corner[2])
            points.append(point)
        
        return points
    
    def calculate_frustum_edges(self, fov_x: float, fov_y: float, near: float, far: float):
        """Calculate frustum edges including lines from camera origin"""
        # Calculate corners at near and far planes
        near_corners = self.calculate_frustum_corners(fov_x, fov_y, near)
        far_corners = self.calculate_frustum_corners(fov_x, fov_y, far)
        
        points = []
        
        # Camera origin (0, 0, 0)
        origin = Point()
        origin.x = 0.0
        origin.y = 0.0
        origin.z = 0.0
        
        # Lines from camera origin to far plane corners
        for corner in far_corners:
            points.append(origin)
            points.append(corner)
        
        # Near plane edges
        for i in range(4):
            points.append(near_corners[i])
            points.append(near_corners[(i + 1) % 4])
        
        # Far plane edges
        for i in range(4):
            points.append(far_corners[i])
            points.append(far_corners[(i + 1) % 4])
        
        # Connect near to far corners
        for i in range(4):
            points.append(near_corners[i])
            points.append(far_corners[i])
        
        return points

def main():
    rclpy.init()
    node = CameraFrustumMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
