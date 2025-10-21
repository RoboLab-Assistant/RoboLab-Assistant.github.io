#!/usr/bin/env python3
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_compressed = DeclareLaunchArgument('use_compressed_image_transport', default_value='false')
    override_intrinsics = DeclareLaunchArgument('override_intrinsics_from_yaml', default_value='false')
    intrinsics_yaml = DeclareLaunchArgument('intrinsics_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','camera_intrinsics.yaml']))
    tags_yaml = DeclareLaunchArgument('tags_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','tags_16h5.yaml']))
    apriltag_settings_yaml = DeclareLaunchArgument('apriltag_settings_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','apriltag_settings.yaml']))

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'rgb_camera.color_profile': '1920x1080x30',
            'enable_sync': 'true',
            'camera_namespace': ''
        }.items()
    )

    cam_override = Node(
        package='cv_module',
        executable='camera_info_override',
        name='camera_info_override',
        parameters=[
            {'intrinsics_yaml': LaunchConfiguration('intrinsics_yaml')},
            {'input_image_topic': '/camera/color/image_raw'},
            {'output_camera_info_topic': '/camera/color/camera_info'},
        ],
        condition=IfCondition(LaunchConfiguration('override_intrinsics_from_yaml'))
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[LaunchConfiguration('apriltag_settings_yaml'), LaunchConfiguration('tags_yaml')],
        remappings=[
            ('image_rect', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info'),
        ]
    )

    draw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('apriltag_draw'),
                                'launch', 'draw.launch.py'])
        ),
        # Tell it where to read images & where to publish the annotated image
        launch_arguments={
            'camera': '/camera',             # base namespace
            'image':  'color/image_raw',     # so it subscribes to /camera/color/image_raw
            # optional: choose transport for viewing (kept raw here)
            'image_transport': 'raw'
        }.items()
    )

    return LaunchDescription([
        use_compressed, override_intrinsics, intrinsics_yaml, tags_yaml, apriltag_settings_yaml,
        realsense, cam_override, apriltag, draw
    ])
