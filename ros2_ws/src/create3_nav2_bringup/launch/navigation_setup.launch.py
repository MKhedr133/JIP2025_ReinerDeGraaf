from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('create3_nav2_bringup')
    pkg_localization = get_package_share_directory('create3_localization_bringup')

    # Configs
    map_yaml = os.path.join(pkg_localization, 'config', 'maps', 'create3_map.yaml')
    nav2_params = os.path.join(pkg_nav2, 'config', 'nav2_params.yaml')

    # --- Nav2 Bringup Launch ---
    nav2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': 'false'
        }.items(),
    )

    # --- Delay bringup a bit until AMCL/map are active ---
    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_nodes])

    return LaunchDescription([delayed_nav2])
