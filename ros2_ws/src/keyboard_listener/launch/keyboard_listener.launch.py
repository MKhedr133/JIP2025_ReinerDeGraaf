from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # Launch the keyboard listener node
        Node(
            package='keyboard_listener',
            executable='keyboard_listener',
            name='keyboard_listener',
            output='screen'
        ),
    ])