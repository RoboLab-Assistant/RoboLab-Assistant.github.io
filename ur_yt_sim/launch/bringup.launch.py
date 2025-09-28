from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 你的 world 文件
    world = PathJoinSubstitution([
        FindPackageShare('ur_yt_sim'), 'worlds', 'my_lab.world'
    ])

    # 用 PathJoinSubstitution 拼出完整的 gazebo 启动文件路径
    gazebo_launch_file = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
    ])

    # 你的 UR5+gripper 启动文件
    ur_launch_file = PathJoinSubstitution([
        FindPackageShare('ur_yt_sim'), 'launch', 'spawn_ur5_camera_gripper_moveit.launch.py'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'world': world}.items()
    )

    ur5_with_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch_file)
    )

    return LaunchDescription([gazebo,ur5_with_gripper])
