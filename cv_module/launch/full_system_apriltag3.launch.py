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
    enable_rviz = DeclareLaunchArgument('enable_rviz', default_value='true')
    enable_rqt = DeclareLaunchArgument('enable_rqt', default_value='true')
    enable_frustum = DeclareLaunchArgument('enable_frustum', default_value='true')
    use_compressed = DeclareLaunchArgument('use_compressed_image_transport', default_value='false')
    bundle_label = DeclareLaunchArgument('bundle_label', default_value='DLS Receptacle')
    lid_tag_size_mm = DeclareLaunchArgument('lid_tag_size_mm', default_value='10.5')
    override_intrinsics = DeclareLaunchArgument('override_intrinsics_from_yaml', default_value='false')
    intrinsics_yaml = DeclareLaunchArgument('intrinsics_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','camera_intrinsics.yaml']))
    tags_yaml = DeclareLaunchArgument('tags_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','tags_16h5.yaml']))
    apriltag3_settings_yaml = DeclareLaunchArgument('apriltag3_settings_yaml', default_value=PathJoinSubstitution([FindPackageShare('cv_module'),'config','apriltag3_settings.yaml']))

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',
            'rgb_camera.color_profile': '1920x1080x30',
            'rgb_camera.color_format': 'RGB8',
            'rgb_camera.enable_color': 'true',
            'enable_sync': 'true',
            'camera_namespace': '',
            'rgb_camera.enable_auto_exposure': 'true',
            'rgb_camera.enable_auto_white_balance': 'true',
            'initial_reset': 'false',
            'unite_imu_method': '0'
        }.items()
    )

    # Optional camera info override
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

    # Static transform to rotate coordinate system for landscape viewing
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_horizontal_tf',
        arguments=[
            '0', '0', '0',  # No translation
            '0', '0', '1.57079632679',  # +90deg around Z axis (180deg - 90deg)
            'camera_color_optical_frame', 'camera_horizontal_frame'
        ]
    )

    # AprilTag 3 detector
    apriltag3 = Node(
        package='cv_module',
        executable='apriltag3_detector',
        name='apriltag3_detector',
        output='screen',
        parameters=[LaunchConfiguration('apriltag3_settings_yaml')]
    )

    # Upstream apriltag_draw overlay (apriltag_detector) with correct topic structure
    draw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('apriltag_draw'), 'launch', 'draw.launch.py'])
        ),
        launch_arguments={
            'camera': '/camera',
            'image': 'color/image_raw',
            'detections': 'tags',
            'annotated': 'image_tags',
            'image_transport': 'raw',
        }.items()
    )

    # Bundle TF filter/republisher
    bundle = Node(
        package='cv_module',
        executable='bundle_tf_filter',
        name='bundle_tf_filter',
        parameters=[
            {'bundle_label': LaunchConfiguration('bundle_label')},
            {'bundle_frame': 'dls_bundle_centroid'},
            {'camera_frame': 'camera_color_optical_frame'},   # RealSense camera frame name
            {'tags_yaml': LaunchConfiguration('tags_yaml')},
            {'pass_through_only': False},
        ],
        remappings=[
            ('detections', '/camera/tags'),
        ]
    )

    # RViz setup
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('cv_module'), 'rviz', 'cv_module.rviz'])],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )

    # Camera frustum marker - adjusted for close-range work (1-50cm)
    frustum = Node(
        package='cv_module',
        executable='camera_frustum_marker',
        name='camera_frustum_marker',
        parameters=[
            {'camera_info_topic': '/camera/color/camera_info'},
            {'frame_id': 'camera_color_optical_frame'},  # Use actual camera frame
            {'near': 0.001},  # 1mm - start at camera origin
            {'far': 0.5},     # 50cm - extend to 50cm
        ],
        condition=IfCondition(LaunchConfiguration('enable_frustum'))
    )

    # Tag coordinate labels node - DISABLED
    # tag_coords = Node(
    #     package='cv_module',
    #     executable='tag_coordinate_markers',
    #     name='tag_coordinate_markers',
    #     parameters=[
    #         {'detections_topic': '/camera/tags'},
    #         {'camera_frame': 'camera_color_optical_frame'},
    #         {'lifetime_sec': 7.5},
    #         {'text_scale': 0.008},  # Smaller scale for 10-50cm distances
    #     ],
    #     output='screen'
    # )

    # rqt image view focused on annotated detections
    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/camera/image_tags'],  # apriltag_draw annotated topic
        condition=IfCondition(LaunchConfiguration('enable_rqt'))
    )

    return LaunchDescription([
        enable_rviz, enable_rqt, enable_frustum, use_compressed,
        bundle_label, lid_tag_size_mm, override_intrinsics,
        intrinsics_yaml, tags_yaml, apriltag3_settings_yaml,
        realsense, static_tf, cam_override, apriltag3, draw, bundle, rviz, frustum, rqt
    ])
