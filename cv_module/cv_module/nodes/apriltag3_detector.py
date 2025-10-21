#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
import cv2
import numpy as np
from cv_bridge import CvBridge
from dt_apriltags import Detector
import dt_apriltags
import tf2_ros
from tf2_ros import TransformBroadcaster
import tf_transformations as tft
import math
from typing import Optional

class AprilTag3Detector(Node):
    def __init__(self):
        super().__init__('apriltag3_detector')
        
        # Parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('detections_topic', '/camera/tags')
        self.declare_parameter('family', 'tag16h5')
        self.declare_parameter('size', 0.014)  # 14mm
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('max_hamming', 0)  # Only accept perfect detections
        
        # Detector parameters (matching dt_apriltags API)
        self.declare_parameter('detector.nthreads', 4)
        self.declare_parameter('detector.quad_decimate', 1.5)
        self.declare_parameter('detector.quad_sigma', 0.2)
        self.declare_parameter('detector.refine_edges', 1)
        self.declare_parameter('detector.decode_sharpening', 0.1)
        self.declare_parameter('detector.debug', 0)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.tag_family = self.get_parameter('family').get_parameter_value().string_value
        self.tag_size = self.get_parameter('size').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.max_hamming = self.get_parameter('max_hamming').get_parameter_value().integer_value
        
        nthreads = self.get_parameter('detector.nthreads').get_parameter_value().integer_value
        quad_decimate = self.get_parameter('detector.quad_decimate').get_parameter_value().double_value
        quad_sigma = self.get_parameter('detector.quad_sigma').get_parameter_value().double_value
        refine_edges = self.get_parameter('detector.refine_edges').get_parameter_value().integer_value
        decode_sharpening = self.get_parameter('detector.decode_sharpening').get_parameter_value().double_value
        debug = self.get_parameter('detector.debug').get_parameter_value().integer_value
        
        # Initialize AprilTag 3 detector with parameters
        self.detector = Detector(
            families=self.tag_family,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=quad_sigma,
            refine_edges=refine_edges,
            decode_sharpening=decode_sharpening,
            debug=debug
        )
        
        # Camera info
        self.camera_info: Optional[CameraInfo] = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.detections_pub = self.create_publisher(AprilTagDetectionArray, self.detections_topic, 10)
        
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, qos)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        
        self.get_logger().info(f"AprilTag 3 detector ready. Family: {self.tag_family}, Size: {self.tag_size}m")
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsic parameters"""
        self.camera_info = msg
        
        # Extract camera matrix
        self.camera_matrix = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]]
        ])
        
        # Extract distortion coefficients
        self.dist_coeffs = np.array(msg.d)
        
        self.get_logger().debug("Camera info received")
    
    def image_callback(self, msg: Image):
        """Process image and detect AprilTags"""
        if self.camera_info is None:
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect tags with pose estimation if camera params available
            if self.camera_matrix is not None:
                # Camera params: fx, fy, cx, cy
                camera_params = [
                    self.camera_matrix[0, 0],  # fx
                    self.camera_matrix[1, 1],  # fy
                    self.camera_matrix[0, 2],  # cx
                    self.camera_matrix[1, 2]   # cy
                ]
                tags = self.detector.detect(
                    gray_image,
                    estimate_tag_pose=True,
                    camera_params=camera_params,
                    tag_size=self.tag_size
                )
            else:
                tags = self.detector.detect(gray_image)
            
            # Create detection array message
            detections_msg = AprilTagDetectionArray()
            detections_msg.header = msg.header
            detections_msg.header.frame_id = self.camera_frame
            
            for tag in tags:
                # Filter by hamming distance to reduce false positives
                if tag.hamming > self.max_hamming:
                    self.get_logger().debug(f"Rejecting tag {tag.tag_id} with hamming distance {tag.hamming}")
                    continue
                
                # Create detection message
                detection = AprilTagDetection()
                detection.id = tag.tag_id
                detection.family = self.tag_family
                detection.hamming = tag.hamming
                detection.decision_margin = tag.decision_margin
                detection.goodness = 0.0  # Not available in AprilTag 3
                
                # Add corners - modify the 4 pre-existing Point objects in place
                for i, corner in enumerate(tag.corners):
                    detection.corners[i].x = float(corner[0])
                    detection.corners[i].y = float(corner[1])
                    # Only set z if the Point has that attribute
                    if hasattr(detection.corners[i], 'z'):
                        detection.corners[i].z = 0.0
                
                # Add center - calculate from corners
                center_x = sum(corner[0] for corner in tag.corners) / 4
                center_y = sum(corner[1] for corner in tag.corners) / 4
                
                # The message might have 'center' or 'centre' (British spelling)
                # Set attributes directly on the existing message field
                # Note: Some messages use 2D points (x, y only), others use 3D (x, y, z)
                if hasattr(detection, 'centre'):
                    detection.centre.x = float(center_x)
                    detection.centre.y = float(center_y)
                    if hasattr(detection.centre, 'z'):
                        detection.centre.z = 0.0
                elif hasattr(detection, 'center'):
                    detection.center.x = float(center_x)
                    detection.center.y = float(center_y)
                    if hasattr(detection.center, 'z'):
                        detection.center.z = 0.0
                
                # Publish TF if pose was estimated (pose_R and pose_t attributes exist)
                if hasattr(tag, 'pose_R') and hasattr(tag, 'pose_t') and tag.pose_R is not None:
                    try:
                        # Convert pose to ROS format
                        pose_stamped = PoseStamped()
                        pose_stamped.header = msg.header
                        pose_stamped.header.frame_id = self.camera_frame
                        
                        # Extract translation from pose_t
                        pose_stamped.pose.position.x = float(tag.pose_t[0])
                        pose_stamped.pose.position.y = float(tag.pose_t[1])
                        pose_stamped.pose.position.z = float(tag.pose_t[2])
                        
                        # Convert rotation matrix (pose_R) to quaternion
                        # Create 4x4 transformation matrix
                        pose_matrix = np.eye(4)
                        pose_matrix[:3, :3] = tag.pose_R
                        pose_matrix[:3, 3] = tag.pose_t.flatten()
                        
                        quaternion = tft.quaternion_from_matrix(pose_matrix)
                        pose_stamped.pose.orientation.x = float(quaternion[0])
                        pose_stamped.pose.orientation.y = float(quaternion[1])
                        pose_stamped.pose.orientation.z = float(quaternion[2])
                        pose_stamped.pose.orientation.w = float(quaternion[3])
                        
                        # Note: apriltag_msgs/AprilTagDetection does NOT have a pose field
                        # Pose is published only as TF
                        
                        # Publish TF if enabled
                        if self.publish_tf:
                            self.publish_tag_tf(tag.tag_id, pose_stamped)
                        
                    except Exception as e:
                        self.get_logger().warn(f"TF publishing failed for tag {tag.tag_id}: {e}")
                
                detections_msg.detections.append(detection)
            
            # Publish detections
            self.detections_pub.publish(detections_msg)
        
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def publish_tag_tf(self, tag_id: int, pose: PoseStamped):
        """Publish TF for individual tag"""
        tf_msg = TransformStamped()
        tf_msg.header = pose.header
        tf_msg.child_frame_id = f"tag16h5:{tag_id}"
        tf_msg.transform.translation.x = pose.pose.position.x
        tf_msg.transform.translation.y = pose.pose.position.y
        tf_msg.transform.translation.z = pose.pose.position.z
        tf_msg.transform.rotation = pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(tf_msg)

def main():
    rclpy.init()
    node = AprilTag3Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
