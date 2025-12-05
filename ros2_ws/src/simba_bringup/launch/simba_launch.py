from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        # # Launch Create3 simulation in Gazebo (Classic)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('irobot_create_gazebo_bringup'),
        #             'launch',
        #             'create3_gazebo.launch.py'
        #         ])
        #     )
        # ),

        # Launch the keyboard listener node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('keyboard_listener'),
                    'launch',
                    'keyboard_listener.launch.py'
                ])
            )
        ),   
        # Launch the main controller node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('main_controller'),
                    'launch',
                    'main_controller.launch.py'
                ])
            )
        ),    
        # Launch the drive controller node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('drive_controller'),
                    'launch',
                    'drive_controller.launch.py'
                ])
            )
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