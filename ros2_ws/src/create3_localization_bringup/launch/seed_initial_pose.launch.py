from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='create3_localization_bringup',
            executable='publish_initialpose',
            name='seed_initial_pose',
            output='screen',
            parameters=[{
                'x': 0.10,     # adjust to your start dock
                'y': 0.05,
                'yaw': 0.0,    # radians
                'frame_id': 'map',
                'delay_sec': 2.0
            }]
        )
    ])
