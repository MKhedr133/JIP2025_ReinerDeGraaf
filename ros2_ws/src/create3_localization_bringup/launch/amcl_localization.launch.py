from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('create3_localization_bringup')

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

    # Lifecycle manager automatically activates the nodes above
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    delayed_lifecycle = TimerAction(
        period=3.0,  # seconds
        actions=[lifecycle_manager]
    )

    return LaunchDescription([map_server, amcl, lifecycle_manager])
