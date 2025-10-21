#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import yaml

class CameraInfoOverride(Node):
    def __init__(self):
        super().__init__('camera_info_override')
        self.declare_parameter('intrinsics_yaml', '')
        self.declare_parameter('input_camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('input_image_topic', '/camera/color/image_raw')
        self.declare_parameter('output_camera_info_topic', '/camera/color/camera_info')

        self.yaml_path = self.get_parameter('intrinsics_yaml').get_parameter_value().string_value
        with open(self.yaml_path, 'r') as f:
            y = yaml.safe_load(f)

        # Build CameraInfo message template
        ci = CameraInfo()
        ci.width = int(y['image_width'])
        ci.height = int(y['image_height'])
        ci.distortion_model = y['distortion_model']
        ci.d = list(map(float, y['distortion_coefficients']['data']))
        ci.k = list(map(float, y['camera_matrix']['data']))
        ci.r = list(map(float, y['rectification_matrix']['data']))
        ci.p = list(map(float, y['projection_matrix']['data']))
        self.ci_template = ci

        self.pub = self.create_publisher(CameraInfo, self.get_parameter('output_camera_info_topic').get_parameter_value().string_value, 10)
        self.sub_img = self.create_subscription(Image, self.get_parameter('input_image_topic').get_parameter_value().string_value, self.img_cb, 10)

        self.get_logger().info(f"Overriding camera_info from {self.yaml_path}")

    def img_cb(self, img: Image):
        ci = CameraInfo()
        ci.header = img.header
        ci.width = self.ci_template.width
        ci.height = self.ci_template.height
        ci.distortion_model = self.ci_template.distortion_model
        ci.d = self.ci_template.d
        ci.k = self.ci_template.k
        ci.r = self.ci_template.r
        ci.p = self.ci_template.p
        self.pub.publish(ci)

def main():
    rclpy.init()
    node = CameraInfoOverride()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
