#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    image_folder = DeclareLaunchArgument('image_folder', default_value='')
    camera_info_yaml = DeclareLaunchArgument('camera_info_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','camera_intrinsics.yaml']))
    apriltag_settings_yaml = DeclareLaunchArgument('apriltag_settings_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','apriltag_settings.yaml']))
    tags_yaml = DeclareLaunchArgument('tags_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','tags_16h5.yaml']))

    # image_publisher
    img_pub = Node(
        package='image_tools',
        executable='cam2image',
        name='cam2image',
        output='screen',
        parameters=[{'burger_mode': False}]
    )
    # Note: for real offline regression use a dedicated folder publisher if needed.

    cam_info_override = Node(
        package='cv_module',
        executable='camera_info_override',
        name='camera_info_override',
        parameters=[
            {'intrinsics_yaml': LaunchConfiguration('camera_info_yaml')},
            {'input_image_topic': '/image'},
            {'output_camera_info_topic': '/camera/color/camera_info'},
        ]
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[LaunchConfiguration('apriltag_settings_yaml'), LaunchConfiguration('tags_yaml')],
        remappings=[
            ('image_rect', '/image'),
            ('camera_info', '/camera/color/camera_info'),
        ]
    )

    draw = Node(
        package='apriltag_draw',
        executable='draw_detections_node',
        name='apriltag_draw',
        output='screen',
        remappings=[
            ('image', '/image'),
            ('detections', '/apriltag/detections'),
            ('annotated', '/apriltag/annotated'),
        ]
    )

    return LaunchDescription([
        image_folder, camera_info_yaml, apriltag_settings_yaml, tags_yaml,
        img_pub, cam_info_override, apriltag, draw
    ])
