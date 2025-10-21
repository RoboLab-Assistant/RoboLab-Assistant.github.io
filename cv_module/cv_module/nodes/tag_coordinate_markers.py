#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft
import math

class TagCoordinateMarkers(Node):
    def __init__(self):
        super().__init__('tag_coordinate_markers')
        
        # Parameters
        self.declare_parameter('detections_topic', '/camera/tags')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('lifetime_sec', 7.5)
        self.declare_parameter('text_scale', 0.008)
        
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lifetime_sec = self.get_parameter('lifetime_sec').get_parameter_value().double_value
        self.text_scale = self.get_parameter('text_scale').get_parameter_value().double_value
        
        # TF buffer and listener
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=self.lifetime_sec))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher
        self.marker_pub = self.create_publisher(Marker, '/cv_module/tag_coordinates', 10)
        
        # Subscriber
        self.sub = self.create_subscription(
            AprilTagDetectionArray, 
            self.detections_topic, 
            self.detections_callback, 
            10
        )
        
        self.get_logger().info("Tag coordinate markers node ready")
    
    def detections_callback(self, msg: AprilTagDetectionArray):
        """Process detections and publish coordinate markers"""
        for detection in msg.detections:
            tag_id = detection.id
            frame_id = f"tag16h5:{tag_id}"
            
            try:
                # Lookup transform from camera to tag
                transform = self.tf_buffer.lookup_transform(
                    self.camera_frame, 
                    frame_id, 
                    rclpy.time.Time()
                )
                
                # Extract position (convert to mm)
                x_mm = transform.transform.translation.x * 1000.0
                y_mm = transform.transform.translation.y * 1000.0
                z_mm = transform.transform.translation.z * 1000.0
                
                # Extract orientation (convert to RPY in degrees)
                quat = [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ]
                roll, pitch, yaw = tft.euler_from_quaternion(quat)
                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)
                
                # Create marker
                marker = Marker()
                marker.header.frame_id = self.camera_frame
                marker.header.stamp = msg.header.stamp
                marker.ns = "tag_coords"
                marker.id = tag_id
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                
                # Position slightly offset from tag
                marker.pose.position.x = transform.transform.translation.x
                marker.pose.position.y = transform.transform.translation.y
                marker.pose.position.z = transform.transform.translation.z + 0.02
                marker.pose.orientation.w = 1.0
                
                # Text content
                marker.text = (
                    f"ID {tag_id}\n"
                    f"({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}mm)\n"
                    f"R:{roll_deg:.1f}° P:{pitch_deg:.1f}° Y:{yaw_deg:.1f}°"
                )
                
                # Appearance
                marker.scale.z = self.text_scale
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                
                # Lifetime
                marker.lifetime = rclpy.duration.Duration(seconds=self.lifetime_sec).to_msg()
                
                self.marker_pub.publish(marker)
                
            except Exception as e:
                self.get_logger().debug(f"Could not get transform for tag {tag_id}: {e}")

def main():
    rclpy.init()
    node = TagCoordinateMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()