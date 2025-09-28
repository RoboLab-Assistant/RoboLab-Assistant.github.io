from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'labworld',
                        '-file', os.path.expanduser('~/.gazebo/models/labworld/model.sdf'),
                        '-x', '-0.5', '-y', '0.85', '-z', '0.92'
                    ],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'cuvette',
                        '-file', os.path.expanduser('~/.gazebo/models/cuvette/model.sdf'),
                        '-x', '0.', '-y', '0.66', '-z', '1.08'
                    ],
                    output='screen'
                )
            ]
        )
    ])
