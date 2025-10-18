from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('create3_localization_bringup')
    slam_cfg = os.path.join(pkg_share, 'config', 'slam_toolbox_localization.yaml')
    map_path = '/work/config/maps/create3_map.yaml'

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_path}]
    )

    slam_loc = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_cfg]
    )

    return LaunchDescription([map_server, slam_loc])
