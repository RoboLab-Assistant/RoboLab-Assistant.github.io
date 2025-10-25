from setuptools import setup, find_packages

package_name = 'cv_module'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', [
            'launch/full_system.launch.py',
            'launch/full_system_apriltag3.launch.py',
            'launch/detector_only.launch.py',
            'launch/offline_test.launch.py',
            'launch/calibrate_camera.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/tags_16h5.yaml',
            'config/apriltag_settings.yaml',
            'config/apriltag3_settings.yaml',
            'config/apriltag3_bundle.yaml',
            'config/camera_intrinsics.yaml',
        ]),
        ('share/' + package_name + '/rviz', ['rviz/cv_module.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='you@example.com',
    description='AprilTag-based CV module for D435 RGB with bundle centroid TF, RViz/rqt visualisation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bundle_tf_filter = cv_module.nodes.bundle_tf_filter:main',
            'camera_frustum_marker = cv_module.nodes.camera_frustum_marker:main',
            'camera_info_override = cv_module.nodes.camera_info_override:main',
            'tag_coordinate_markers = cv_module.nodes.tag_coordinate_markers:main',
            'apriltag3_detector = cv_module.nodes.apriltag3_detector:main',
            'camera_calibrator = cv_module.nodes.camera_calibrator:main',
        ],
    },
)
