#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from apriltag_msgs.msg import AprilTagDetectionArray
import tf_transformations as tft
import tf2_ros
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from builtin_interfaces.msg import Time
import yaml
import os
import math
from typing import Dict, List, Optional

def dict_to_quat_xyzw(d):
    return [d[0], d[1], d[2], d[3]]

class BundleTFFilter(Node):
    def __init__(self):
        super().__init__('bundle_tf_filter')
        self.declare_parameter('bundle_label', 'DLS Receptacle')
        self.declare_parameter('bundle_frame', 'dls_bundle_centroid')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tags_yaml', '')
        self.declare_parameter('pass_through_only', False)  # if another node already publishes bundle TF
        self.declare_parameter('publish_tag_pose_topics', True)

        self.bundle_label = self.get_parameter('bundle_label').get_parameter_value().string_value
        self.bundle_frame = self.get_parameter('bundle_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tags_yaml_path = self.get_parameter('tags_yaml').get_parameter_value().string_value
        self.pass_through_only = self.get_parameter('pass_through_only').get_parameter_value().bool_value
        self.publish_tag_pose_topics = self.get_parameter('publish_tag_pose_topics').get_parameter_value().bool_value

        # Load bundle config from YAML (under apriltag.ros__parameters.bundle)
        self.bundle_ids: List[int] = []
        self.tag_to_bundle: Dict[int, Dict] = {}
        if self.tags_yaml_path and os.path.exists(self.tags_yaml_path):
            with open(self.tags_yaml_path, 'r') as f:
                y = yaml.safe_load(f)
            try:
                b = y['apriltag']['ros__parameters']['bundle']
                self.bundle_frame = b.get('name', self.bundle_frame)
                self.bundle_ids = list(b['ids'])
                # poses: { id: {translation: [x,y,z], rotation_xyzw: [x,y,z,w]} }
                self.tag_to_bundle = b['poses']
                self.get_logger().info(f"Loaded bundle config with IDs {self.bundle_ids} for frame '{self.bundle_frame}'.")
            except Exception as e:
                self.get_logger().warn(f"No bundle section found in {self.tags_yaml_path}: {e}")
        else:
            self.get_logger().warn("No tags YAML provided; bundle functionality limited.")

        qos = QoSProfile(depth=10)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE

        # Subscribe to apriltag_ros detections for margins and IDs
        self.sub = self.create_subscription(AprilTagDetectionArray, 'detections', self.detections_cb, qos)

        # TF interfaces
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=7.5))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Bundle label marker publisher
        self.marker_pub = self.create_publisher(Marker, '/cv_module/bundle_label', 10)

        self.last_bundle_pub_time: Optional[Time] = None
        self.bundle_hold_sec = 0.0  # no hold by default (no stale TF)

        self.get_logger().info("bundle_tf_filter ready.")

    def detections_cb(self, msg: AprilTagDetectionArray):
        # Collect all visible bundle tags
        visible_tags = []
        for det in msg.detections:
            tag_id = int(det.id)
            if tag_id not in self.bundle_ids:
                # Optionally publish PoseStamped for non-bundle selected tags (e.g., cuvette/lid)
                if self.publish_tag_pose_topics:
                    self.publish_pose_topic_from_tf(tag_id, msg.header.stamp)
                continue

            # Lookup TF from camera -> this tag frame
            frame = f"tag16h5:{tag_id}"
            try:
                t = self.tf_buffer.lookup_transform(self.camera_frame, frame, rclpy.time.Time())
                visible_tags.append((tag_id, float(det.decision_margin), t))
            except Exception as e:
                # No TF yet for this tag
                continue

            if self.publish_tag_pose_topics:
                self.publish_pose_topic_from_tf(tag_id, msg.header.stamp)

        if not visible_tags:
            return

        # Check if bundle TF is already published by apriltag3_detector
        # If so, we just need to republish the marker and pose topics
        try:
            bundle_tf = self.tf_buffer.lookup_transform(self.camera_frame, self.bundle_frame, rclpy.time.Time())
            
            # Bundle TF already exists (published by apriltag3_detector with bundle detection)
            # Just publish marker and pose topics
            self.publish_label_marker_from_tf(bundle_tf)
            
            if self.publish_tag_pose_topics:
                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = self.camera_frame
                ps.pose.position.x = bundle_tf.transform.translation.x
                ps.pose.position.y = bundle_tf.transform.translation.y
                ps.pose.position.z = bundle_tf.transform.translation.z
                ps.pose.orientation = bundle_tf.transform.rotation
                self.get_or_create_pub('/cv_module/dls_bundle_centroid/pose').publish(ps)
            
            self.get_logger().debug("Bundle TF already published by apriltag3_detector")
            return
            
        except Exception:
            # Bundle TF not available, fall back to legacy weighted averaging method
            self.get_logger().debug("Bundle TF not available, using legacy weighted averaging")
            pass
        
        # Legacy bundle calculation (fallback when bundle detection is not available)
        if not self.pass_through_only:
            # Initialize tracking variables
            if not hasattr(self, 'bundle_history'):
                self.bundle_history = []
                self.max_history = 10
                self.outlier_threshold = 0.05  # 5cm threshold for outlier detection
                self.min_tags_for_averaging = 2  # Minimum tags needed for averaging
            
            # Calculate bundle center from all visible tags
            bundle_estimates = []
            total_weight = 0.0
            
            for tag_id, dm, t_cam_tag in visible_tags:
                # Get the bundle transform for this tag from the YAML configuration
                tb = self.tag_to_bundle.get(str(tag_id)) or self.tag_to_bundle.get(tag_id)
                if tb is None:
                    continue
                
                # Convert to 4x4 matrices
                T_cam_tag = t_to_mat(t_cam_tag.transform.translation, t_cam_tag.transform.rotation)
                T_tag_bundle = t_to_mat(tb['translation'], tb['rotation_xyzw'])
                T_cam_bundle = T_cam_tag @ T_tag_bundle
                
                # Extract bundle center position in camera frame
                center_x = T_cam_bundle[0, 3]
                center_y = T_cam_bundle[1, 3]
                center_z = T_cam_bundle[2, 3]
                
                # Use decision margin as weight (higher = more reliable)
                weight = max(dm, 0.1)  # Ensure minimum weight
                
                bundle_estimates.append({
                    'position': [center_x, center_y, center_z],
                    'weight': weight,
                    'tag_id': tag_id,
                    'orientation': t_cam_tag.transform.rotation
                })
                total_weight += weight
            
            if not bundle_estimates:
                self.get_logger().warn_throttle(2.0, "No valid bundle estimates available")
                return
            
            # Calculate weighted average of all estimates
            if len(bundle_estimates) >= self.min_tags_for_averaging:
                # Multi-tag estimation: weighted average
                weighted_x = 0.0
                weighted_y = 0.0
                weighted_z = 0.0
                
                for estimate in bundle_estimates:
                    pos = estimate['position']
                    weight = estimate['weight']
                    
                    weighted_x += pos[0] * weight
                    weighted_y += pos[1] * weight
                    weighted_z += pos[2] * weight
                
                center_x = weighted_x / total_weight
                center_y = weighted_y / total_weight
                center_z = weighted_z / total_weight
                
                # Use orientation from the most reliable tag
                best_estimate = max(bundle_estimates, key=lambda x: x['weight'])
                best_orientation = best_estimate['orientation']
                
                self.get_logger().debug(f"Multi-tag estimation: {len(bundle_estimates)} tags, total weight: {total_weight:.1f}")
            else:
                # Single tag fallback: use the best available tag
                best_estimate = max(bundle_estimates, key=lambda x: x['weight'])
                center_x, center_y, center_z = best_estimate['position']
                best_orientation = best_estimate['orientation']
                
                self.get_logger().debug(f"Single-tag fallback: tag {best_estimate['tag_id']}, weight: {best_estimate['weight']:.1f}")
            
            # Outlier detection and rejection
            current_pos = [center_x, center_y, center_z]
            is_outlier = False
            
            if len(self.bundle_history) > 0:
                # Check if current measurement is an outlier
                last_pos = self.bundle_history[-1]
                distance = math.sqrt(
                    (center_x - last_pos[0])**2 + 
                    (center_y - last_pos[1])**2 + 
                    (center_z - last_pos[2])**2
                )
                
                if distance > self.outlier_threshold:
                    is_outlier = True
                    self.get_logger().debug(f"Outlier detected: distance={distance:.4f}m, threshold={self.outlier_threshold}m")
            
            # Only add to history if not an outlier
            if not is_outlier:
                self.bundle_history.append(current_pos)
                
                # Keep only recent history
                if len(self.bundle_history) > self.max_history:
                    self.bundle_history.pop(0)
            
            # Apply weighted smoothing (more weight on recent measurements)
            if len(self.bundle_history) >= 2:
                # Use weighted average with exponential decay
                smoothed_x = 0.0
                smoothed_y = 0.0
                smoothed_z = 0.0
                total_weight = 0.0
                
                for i, pos in enumerate(self.bundle_history):
                    # Exponential weight: more recent = higher weight
                    weight = math.exp(-0.3 * (len(self.bundle_history) - 1 - i))
                    
                    smoothed_x += pos[0] * weight
                    smoothed_y += pos[1] * weight
                    smoothed_z += pos[2] * weight
                    total_weight += weight
                
                if total_weight > 0:
                    smoothed_x /= total_weight
                    smoothed_y /= total_weight
                    smoothed_z /= total_weight
                else:
                    smoothed_x, smoothed_y, smoothed_z = center_x, center_y, center_z
            else:
                smoothed_x = center_x
                smoothed_y = center_y
                smoothed_z = center_z
            
            # Create bundle transform at the smoothed centroid
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self.camera_frame
            tf_msg.child_frame_id = self.bundle_frame
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.transform.translation.x = smoothed_x
            tf_msg.transform.translation.y = smoothed_y
            tf_msg.transform.translation.z = smoothed_z
            tf_msg.transform.rotation = best_orientation
            
            self.tf_broadcaster.sendTransform(tf_msg)

            # Publish marker label near the bundle
            self.publish_label_marker(tf_msg)

            # Publish PoseStamped for bundle
            if self.publish_tag_pose_topics:
                ps = PoseStamped()
                ps.header.stamp = msg.header.stamp
                ps.header.frame_id = self.camera_frame
                ps.pose.position.x = tf_msg.transform.translation.x
                ps.pose.position.y = tf_msg.transform.translation.y
                ps.pose.position.z = tf_msg.transform.translation.z
                ps.pose.orientation = tf_msg.transform.rotation
                self.get_or_create_pub('/cv_module/dls_bundle_centroid/pose').publish(ps)

    def get_or_create_pub(self, topic):
        if not hasattr(self, '_pub_cache'):
            self._pub_cache = {}
        if topic not in self._pub_cache:
            self._pub_cache[topic] = self.create_publisher(PoseStamped, topic, 10)
        return self._pub_cache[topic]

    def publish_pose_topic_from_tf(self, tag_id: int, stamp: Time):
        frame = f"tag16h5:{tag_id}"
        try:
            t = self.tf_buffer.lookup_transform(self.camera_frame, frame, rclpy.time.Time())
        except Exception:
            return
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.camera_frame
        ps.pose.position.x = t.transform.translation.x
        ps.pose.position.y = t.transform.translation.y
        ps.pose.position.z = t.transform.translation.z
        ps.pose.orientation = t.transform.rotation
        self.get_or_create_pub(f"/cv_module/tag_{tag_id}/pose").publish(ps)

    def publish_label_marker(self, tf_msg: TransformStamped):
        m = Marker()
        m.header.frame_id = tf_msg.header.frame_id
        m.header.stamp = tf_msg.header.stamp
        m.ns = "cv_module"
        m.id = 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = tf_msg.transform.translation.x
        m.pose.position.y = tf_msg.transform.translation.y
        m.pose.position.z = tf_msg.transform.translation.z + 0.02
        m.text = self.bundle_label
        m.scale.z = 0.02  # 2 cm text height
        self.marker_pub.publish(m)
    
    def publish_label_marker_from_tf(self, tf_msg: TransformStamped):
        """Publish marker label from existing TF transform"""
        m = Marker()
        m.header.frame_id = tf_msg.header.frame_id
        m.header.stamp = tf_msg.header.stamp
        m.ns = "cv_module"
        m.id = 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = tf_msg.transform.translation.x
        m.pose.position.y = tf_msg.transform.translation.y
        m.pose.position.z = tf_msg.transform.translation.z + 0.02
        m.text = self.bundle_label
        m.scale.z = 0.02  # 2 cm text height
        self.marker_pub.publish(m)

def t_to_mat(trans, quat):
    if hasattr(trans, 'x'):
        tx, ty, tz = trans.x, trans.y, trans.z
        qx, qy, qz, qw = quat.x, quat.y, quat.z, quat.w
    else:
        tx, ty, tz = trans
        qx, qy, qz, qw = quat
    T = tft.quaternion_matrix([qx, qy, qz, qw])
    T[0,3], T[1,3], T[2,3] = tx, ty, tz
    return T

def mat_to_tf(T, parent, child, stamp):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.header.stamp = stamp
    t.transform.translation.x = float(T[0,3])
    t.transform.translation.y = float(T[1,3])
    t.transform.translation.z = float(T[2,3])
    q = tft.quaternion_from_matrix(T)
    t.transform.rotation.x = float(q[0])
    t.transform.rotation.y = float(q[1])
    t.transform.rotation.z = float(q[2])
    t.transform.rotation.w = float(q[3])
    return t

def main():
    rclpy.init()
    node = BundleTFFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
