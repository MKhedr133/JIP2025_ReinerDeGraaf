from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('create3_localization_bringup')
    map_yaml = os.path.join(pkg_share, 'config', 'maps', 'create3_map.yaml')
    map_data = os.path.join(pkg_share, 'config', 'maps', 'create3_map.data')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}]
    )

    slam_loc = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'mode': 'localization',
            'map_file_name': map_data,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'odom_topic': '/odometry/filtered',
            'scan_topic': '/scan',
            'map_start_at_dock': True
        }]
    )

    return LaunchDescription([map_server, slam_loc])
