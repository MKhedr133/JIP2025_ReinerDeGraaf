"""
Launch all three nodes
======================
Starts: raw reader → filter → csv logger.

Usage examples
- Default (USB0, 19200 baud):
  ros2 launch scintillator_lb124 scintillator.launch.py
- Custom port & threshold:
  ros2 launch scintillator_lb124 scintillator.launch.py port:=/dev/ttyUSB1 baud:=19200 threshold:=25.0
- Custom CSV directory:
  ros2 launch scintillator_lb124 scintillator.launch.py output_dir:=/home/user/logs
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_output_dir = os.path.join(
        get_package_share_directory('scintillator_lb124'), 'csv'
    )

    port = DeclareLaunchArgument('port', default_value=TextSubstitution(text='/dev/ttyUSB0'))
    baud = DeclareLaunchArgument('baud', default_value=TextSubstitution(text='19200'))
    threshold = DeclareLaunchArgument('threshold', default_value=TextSubstitution(text='20.0'))
    output_dir = DeclareLaunchArgument('output_dir', default_value=TextSubstitution(text=default_output_dir))

    raw = Node(
        package='scintillator_lb124',
        executable='scintillator_raw_node',
        name='scintillator_raw',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'timeout_s': 1.0,
        }],
        output='screen',
    )

    filt = Node(
        package='scintillator_lb124',
        executable='scintillator_filtered_node',
        name='scintillator_filter',
        parameters=[{
            'threshold': LaunchConfiguration('threshold'),
            'cps_index': 1,
        }],
        output='screen',
    )

    csv = Node(
        package='scintillator_lb124',
        executable='scintillator_csv_node',
        name='scintillator_csv',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'add_header': True,
        }],
        output='screen',
    )

    statistics = Node(
        package='scintillator_lb124',
        executable='cps_statistics',
        name='cps_statistics',
        output='screen',
    )

    return LaunchDescription([port, baud, threshold, output_dir, raw, filt, csv, statistics])