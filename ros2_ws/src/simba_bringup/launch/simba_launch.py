from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Path to the parameter file
    param_file = PathJoinSubstitution([
        FindPackageShare('main_controller'),
        'config',
        'autonomy_params.yaml'
    ])
    # Declare a launch argument for the autonomy mode
    autonomy_mode_arg = DeclareLaunchArgument(
        'autonomy_mode', default_value='basic',
        description='Which autonomous controller to run. Options: basic, gradient, double_sweep, gradient_double_sweep'
    )

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

        # Add the launch argument to the launch description
        autonomy_mode_arg,
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
            executable=LaunchConfiguration('autonomy_mode'),
            name='autonomous_controller',
            output='screen',
            parameters=[param_file]
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