from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('create3_localization_bringup')

    # Use the map you already installed into the package
    map_yaml = os.path.join(pkg_share, 'config', 'maps', 'create3_map.yaml')
    amcl_yaml = os.path.join(pkg_share, 'config', 'amcl_params.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml]
    )

    return LaunchDescription([map_server, amcl])
