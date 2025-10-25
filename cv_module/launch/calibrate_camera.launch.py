#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    tags_yaml = DeclareLaunchArgument(
        'tags_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('cv_module'),
            'config',
            'tags_16h5.yaml'
        ])
    )
    
    output_yaml = DeclareLaunchArgument(
        'output_yaml',
        default_value='camera_intrinsics_calibrated.yaml',
        description='Output file for calibrated camera parameters'
    )
    
    num_images = DeclareLaunchArgument(
        'num_images',
        default_value='25',
        description='Target number of calibration images to collect'
    )
    
    tag_size = DeclareLaunchArgument(
        'tag_size',
        default_value='0.0105',
        description='AprilTag size in meters (0.0105 = 10.5mm)'
    )

    # Launch RealSense camera
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'rgb_camera.color_profile': '1920x1080x30',
            'enable_sync': 'true',
            'camera_namespace': '',
            'rgb_camera.enable_auto_exposure': 'false',  # Disable auto exposure for consistency
            'rgb_camera.enable_auto_white_balance': 'false',
        }.items()
    )

    # Camera calibrator node
    calibrator = Node(
        package='cv_module',
        executable='camera_calibrator',
        name='camera_calibrator',
        output='screen',
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'bundle_yaml': LaunchConfiguration('tags_yaml'),
            'output_yaml': LaunchConfiguration('output_yaml'),
            'num_images': LaunchConfiguration('num_images'),
            'tag_size': LaunchConfiguration('tag_size'),
            'show_preview': True,
        }]
    )

    return LaunchDescription([
        tags_yaml,
        output_yaml,
        num_images,
        tag_size,
        realsense,
        calibrator
    ])

