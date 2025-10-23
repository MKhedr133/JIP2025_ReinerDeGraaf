from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch Create3 simulation in Gazebo (Classic)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('irobot_create_gazebo_bringup'),
                    'launch',
                    'create3_gazebo.launch.py'
                ])
            )
        ),
        # Launch the keyboard listener node
        Node(
            package='keyboard_listener',
            executable='keyboard_listener',
            name='keyboard_listener',
            output='screen'
        ),
        # Launch the main controller node
        Node(
            package='main_controller',
            executable='main_controller',
            name='main_controller',
            output='screen'
        ),
        # Launch the drive controller node
        Node(
            package='drive_controller',
            executable='drive_controller',
            name='drive_controller',
            output='screen'
        ),
        # Launch the autonomous controller node
        Node(
            package='main_controller',
            executable='autonomous_controller',
            name='autonomous_controller',
            output='screen'
        ),
        # Launch the scintillator nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('scintillator_lb124'),
                    'launch',
                    'scintillator.launch.py'
                ])
            )
        ),

    ])