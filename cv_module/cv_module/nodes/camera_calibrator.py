#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from dt_apriltags import Detector
import yaml
from pathlib import Path
import tf_transformations as tft
from typing import List, Dict

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        
        # Parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('bundle_yaml', '')
        self.declare_parameter('output_yaml', 'camera_intrinsics_calibrated.yaml')
        self.declare_parameter('num_images', 25)
        self.declare_parameter('tag_size', 0.014)  # 14mm tags
        self.declare_parameter('show_preview', True)
        
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.bundle_yaml_path = self.get_parameter('bundle_yaml').get_parameter_value().string_value
        self.output_yaml = self.get_parameter('output_yaml').get_parameter_value().string_value
        self.num_images_needed = self.get_parameter('num_images').get_parameter_value().integer_value
        self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value
        self.show_preview = self.get_parameter('show_preview').get_parameter_value().bool_value
        
        # Load bundle configuration
        self.bundle_geometry = self.load_bundle_geometry(self.bundle_yaml_path)
        
        # Initialize detector
        self.detector = Detector(
            families='tag16h5',
            nthreads=4,
            quad_decimate=1.0,  # Use full resolution for calibration
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25
        )
        
        # Storage for calibration data
        self.object_points_list = []  # 3D points in world coordinates
        self.image_points_list = []   # 2D points in image coordinates
        self.image_size = None
        self.images_collected = 0
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Camera Calibrator Started")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Bundle tags loaded: {list(self.bundle_geometry.keys())}")
        self.get_logger().info(f"Target images: {self.num_images_needed}")
        self.get_logger().info(f"Tag size: {self.tag_size * 1000:.1f}mm")
        self.get_logger().info("")
        self.get_logger().info("Instructions:")
        self.get_logger().info("  1. Show the AprilTag bundle to the camera")
        self.get_logger().info("  2. Move it to different positions, angles, and distances")
        self.get_logger().info("  3. Press SPACE when you see a good detection to capture")
        self.get_logger().info("  4. Press 'q' to quit and calibrate with collected images")
        self.get_logger().info("  5. Press 'r' to reset and start over")
        self.get_logger().info("=" * 70)
        
        self.current_frame = None
        self.current_detections = []
        self.capture_requested = False
        self.quit_requested = False
        self.reset_requested = False
        
    def load_bundle_geometry(self, yaml_path: str) -> Dict[int, np.ndarray]:
        """
        Load bundle configuration and convert to 3D corner positions.
        Returns dict mapping tag_id -> 4x3 array of corner positions in meters.
        """
        if not yaml_path or not Path(yaml_path).exists():
            self.get_logger().error(f"Bundle YAML not found: {yaml_path}")
            raise FileNotFoundError(f"Bundle YAML not found: {yaml_path}")
        
        with open(yaml_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        bundle_config = config['apriltag']['ros__parameters']['bundle']
        bundle_ids = bundle_config['ids']
        poses = bundle_config['poses']
        
        self.get_logger().info(f"Loading bundle from: {yaml_path}")
        
        # For each tag, compute the 3D positions of its 4 corners in world frame
        bundle_geometry = {}
        
        for tag_id in bundle_ids:
            # Get tag pose (center) relative to bundle origin
            pose_config = poses[tag_id] if isinstance(tag_id, str) else poses[str(tag_id)]
            translation = pose_config['translation']
            rotation_xyzw = pose_config['rotation_xyzw']
            
            # Build transformation matrix from bundle origin to tag center
            T_bundle_tag = self.pose_to_matrix(translation, rotation_xyzw)
            
            # Define tag corners in tag's local frame
            # Tag coordinate system: origin at center, +X right, +Y down, +Z forward
            # Corners go counter-clockwise from bottom-left when viewing tag
            half_size = self.tag_size / 2.0
            corners_local = np.array([
                [-half_size, -half_size, 0.0],  # Top-left
                [ half_size, -half_size, 0.0],  # Top-right
                [ half_size,  half_size, 0.0],  # Bottom-right
                [-half_size,  half_size, 0.0],  # Bottom-left
            ])
            
            # Transform corners to bundle frame
            corners_world = []
            for corner_local in corners_local:
                corner_local_h = np.append(corner_local, 1.0)  # Homogeneous coords
                corner_world_h = T_bundle_tag @ corner_local_h
                corners_world.append(corner_world_h[:3])
            
            bundle_geometry[tag_id] = np.array(corners_world, dtype=np.float32)
            
            self.get_logger().debug(f"Tag {tag_id}: center at {translation}")
        
        return bundle_geometry
    
    def pose_to_matrix(self, translation: List[float], quat_xyzw: List[float]) -> np.ndarray:
        """Convert translation and quaternion to 4x4 transformation matrix."""
        T = tft.quaternion_matrix(quat_xyzw)
        T[0:3, 3] = translation
        return T
    
    def image_callback(self, msg: Image):
        """Process incoming images."""
        try:
            # Convert to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            if self.image_size is None:
                self.image_size = (cv_image.shape[1], cv_image.shape[0])
                self.get_logger().info(f"Image size: {self.image_size[0]}x{self.image_size[1]}")
            
            # Detect tags (no pose estimation yet - we're calibrating!)
            detections = self.detector.detect(gray_image)
            
            # Store for visualization
            self.current_frame = cv_image.copy()
            self.current_detections = detections
            
            # Filter to only bundle tags
            bundle_detections = [d for d in detections if d.tag_id in self.bundle_geometry]
            
            # Draw detections
            self.draw_detections(self.current_frame, bundle_detections)
            
            # Check if user requested capture
            if self.capture_requested and len(bundle_detections) >= 4:
                self.capture_image(bundle_detections)
                self.capture_requested = False
            
            # Show preview
            if self.show_preview:
                status_text = f"Images: {self.images_collected}/{self.num_images_needed} | Tags: {len(bundle_detections)}"
                cv2.putText(self.current_frame, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                if len(bundle_detections) >= 4:
                    cv2.putText(self.current_frame, "SPACE: Capture | Q: Calibrate | R: Reset", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    cv2.putText(self.current_frame, f"Show at least 4 tags ({len(bundle_detections)} visible)", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                cv2.imshow("Camera Calibration", self.current_frame)
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord(' '):  # Space to capture
                    if len(bundle_detections) >= 4:
                        self.capture_requested = True
                elif key == ord('q'):  # Q to quit and calibrate
                    self.quit_requested = True
                elif key == ord('r'):  # R to reset
                    self.reset_calibration()
            
            # Check if we have enough images
            if self.quit_requested and self.images_collected >= 10:
                self.perform_calibration()
                cv2.destroyAllWindows()
                rclpy.shutdown()
            elif self.images_collected >= self.num_images_needed:
                self.get_logger().info("Target images reached! Press 'q' to calibrate or keep adding more.")
        
        except Exception as e:  # pylint: disable=broad-except
            self.get_logger().error(f"Error processing image: {e}")
    
    def draw_detections(self, image: np.ndarray, detections: List):
        """Draw detected tags on image."""
        for detection in detections:
            # Draw corners
            corners = detection.corners.astype(int)
            for i in range(4):
                cv2.line(image, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
            
            # Draw center and ID
            center = detection.center.astype(int)
            cv2.circle(image, tuple(center), 5, (0, 0, 255), -1)
            cv2.putText(image, str(detection.tag_id), 
                       (center[0] + 10, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    def capture_image(self, detections: List):
        """Capture current detections for calibration."""
        object_points = []
        image_points = []
        
        for detection in detections:
            tag_id = detection.tag_id
            if tag_id not in self.bundle_geometry:
                continue
            
            # Get 3D corners from bundle geometry
            corners_3d = self.bundle_geometry[tag_id]
            
            # Get 2D corners from detection
            # dt_apriltags corners are in order: [bottom-left, bottom-right, top-right, top-left]
            # We need: [top-left, top-right, bottom-right, bottom-left]
            corners_2d = detection.corners[[3, 2, 1, 0], :]  # Reorder to match
            
            object_points.extend(corners_3d)
            image_points.extend(corners_2d)
        
        if len(object_points) >= 16:  # At least 4 tags * 4 corners
            self.object_points_list.append(np.array(object_points, dtype=np.float32))
            self.image_points_list.append(np.array(image_points, dtype=np.float32))
            self.images_collected += 1
            
            self.get_logger().info(f"✓ Captured image {self.images_collected} with {len(detections)} tags "
                                  f"({len(object_points)} points)")
            
            # Visual feedback
            if self.show_preview:
                feedback_frame = self.current_frame.copy()
                cv2.putText(feedback_frame, "CAPTURED!", (self.image_size[0]//2 - 150, self.image_size[1]//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)
                cv2.imshow("Camera Calibration", feedback_frame)
                cv2.waitKey(500)
        else:
            self.get_logger().warn(f"Not enough points: {len(object_points)} (need at least 16)")
    
    def reset_calibration(self):
        """Reset all collected data."""
        self.object_points_list = []
        self.image_points_list = []
        self.images_collected = 0
        self.get_logger().info("Calibration data reset.")
    
    def perform_calibration(self):
        """Perform camera calibration using collected data."""
        if self.images_collected < 10:
            self.get_logger().error(f"Not enough images for calibration: {self.images_collected} (need at least 10)")
            return
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Starting calibration...")
        self.get_logger().info(f"Using {self.images_collected} images")
        
        # Perform calibration
        flags = cv2.CALIB_RATIONAL_MODEL  # Use 8-parameter model (k1-k6, p1, p2)
        
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points_list,
            self.image_points_list,
            self.image_size,
            None,
            None,
            flags=flags
        )
        
        if not ret:
            self.get_logger().error("Calibration failed!")
            return
        
        self.get_logger().info(f"✓ Calibration successful! RMS reprojection error: {ret:.4f} pixels")
        
        # Calculate per-image errors
        total_error = 0
        total_points = 0
        for i in range(len(self.object_points_list)):
            imgpoints2, _ = cv2.projectPoints(
                self.object_points_list[i],
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist_coeffs
            )
            error = cv2.norm(self.image_points_list[i], imgpoints2.reshape(-1, 2), cv2.NORM_L2)
            n = len(self.object_points_list[i])
            total_error += error**2
            total_points += n
        
        mean_error = np.sqrt(total_error / total_points)
        self.get_logger().info(f"Mean reprojection error: {mean_error:.4f} pixels")
        
        # Display results
        self.get_logger().info("")
        self.get_logger().info("Camera Matrix (K):")
        self.get_logger().info(f"  fx = {camera_matrix[0, 0]:.2f} pixels")
        self.get_logger().info(f"  fy = {camera_matrix[1, 1]:.2f} pixels")
        self.get_logger().info(f"  cx = {camera_matrix[0, 2]:.2f} pixels")
        self.get_logger().info(f"  cy = {camera_matrix[1, 2]:.2f} pixels")
        self.get_logger().info("")
        self.get_logger().info("Distortion Coefficients:")
        self.get_logger().info(f"  k1 = {dist_coeffs[0, 0]:.6f}")
        self.get_logger().info(f"  k2 = {dist_coeffs[0, 1]:.6f}")
        self.get_logger().info(f"  p1 = {dist_coeffs[0, 2]:.6f}")
        self.get_logger().info(f"  p2 = {dist_coeffs[0, 3]:.6f}")
        self.get_logger().info(f"  k3 = {dist_coeffs[0, 4]:.6f}")
        if dist_coeffs.shape[1] > 5:
            self.get_logger().info(f"  k4 = {dist_coeffs[0, 5]:.6f}")
            self.get_logger().info(f"  k5 = {dist_coeffs[0, 6]:.6f}")
            self.get_logger().info(f"  k6 = {dist_coeffs[0, 7]:.6f}")
        
        # Save to YAML file
        self.save_calibration(camera_matrix, dist_coeffs)
        
        self.get_logger().info("=" * 70)
    
    def save_calibration(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """Save calibration to YAML file in ROS camera_info format."""
        # Build rectification matrix (identity for monocular)
        rectification_matrix = np.eye(3)
        
        # Build projection matrix [K | 0]
        projection_matrix = np.zeros((3, 4))
        projection_matrix[:3, :3] = camera_matrix
        
        # Create YAML structure matching camera_intrinsics.yaml format
        calibration_data = {
            'image_width': int(self.image_size[0]),
            'image_height': int(self.image_size[1]),
            'camera_name': 'd435',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [
                    float(camera_matrix[0, 0]), float(camera_matrix[0, 1]), float(camera_matrix[0, 2]),
                    float(camera_matrix[1, 0]), float(camera_matrix[1, 1]), float(camera_matrix[1, 2]),
                    float(camera_matrix[2, 0]), float(camera_matrix[2, 1]), float(camera_matrix[2, 2])
                ]
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': 5,
                'data': [
                    float(dist_coeffs[0, 0]),
                    float(dist_coeffs[0, 1]),
                    float(dist_coeffs[0, 2]),
                    float(dist_coeffs[0, 3]),
                    float(dist_coeffs[0, 4])
                ]
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': [
                    float(rectification_matrix[0, 0]), float(rectification_matrix[0, 1]), float(rectification_matrix[0, 2]),
                    float(rectification_matrix[1, 0]), float(rectification_matrix[1, 1]), float(rectification_matrix[1, 2]),
                    float(rectification_matrix[2, 0]), float(rectification_matrix[2, 1]), float(rectification_matrix[2, 2])
                ]
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': [
                    float(projection_matrix[0, 0]), float(projection_matrix[0, 1]), 
                    float(projection_matrix[0, 2]), float(projection_matrix[0, 3]),
                    float(projection_matrix[1, 0]), float(projection_matrix[1, 1]), 
                    float(projection_matrix[1, 2]), float(projection_matrix[1, 3]),
                    float(projection_matrix[2, 0]), float(projection_matrix[2, 1]), 
                    float(projection_matrix[2, 2]), float(projection_matrix[2, 3])
                ]
            }
        }
        
        with open(self.output_yaml, 'w', encoding='utf-8') as f:
            yaml.dump(calibration_data, f, default_flow_style=False, sort_keys=False)
        
        self.get_logger().info(f"✓ Calibration saved to: {self.output_yaml}")
        self.get_logger().info("  To use it, update your launch file with:")
        self.get_logger().info("    override_intrinsics_from_yaml:=true")
        self.get_logger().info(f"    intrinsics_yaml:={Path(self.output_yaml).absolute()}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

