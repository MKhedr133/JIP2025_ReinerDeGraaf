from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Package paths ---
    lidar_pkg = get_package_share_directory('create3_lidar_slam')
    localization_pkg = get_package_share_directory('create3_localization_bringup')

    # --- Config paths ---
    map_yaml = os.path.join(localization_pkg, 'config', 'maps', 'create3_home_map.yaml')
    amcl_yaml = os.path.join(localization_pkg, 'config', 'amcl_params.yaml')

    # --- Launch: Sensors (LiDAR + static TF + EKF) ---
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'sensors_launch.py')
        )
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'ekf_launch.py')
        )
    )

    # --- Nodes: Map server & AMCL ---
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

    # --- Lifecycle manager to activate nodes automatically ---
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
        period=3.0,  # give sensors time to come up
        actions=[lifecycle_manager]
    )

    # --- RViz (optional, can remove if you prefer manual) ---
    rviz_config = os.path.join(lidar_pkg, 'rviz', 'create3_lidar_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # --- Automatically publish the initial 2d pose esitmate ---
    init_pose = Node(
        package='state_machine',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            # set sensible defaults for your map; can be overridden via launch args if needed
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_yaw_deg': 0.0,
            'frame_id': 'map',
            'wait_for_map_timeout_sec': 30.0,
        }]
    )

    # --- Return full launch description ---
    return LaunchDescription([
        sensors_launch,
        ekf_launch,
        map_server,
        amcl,
        delayed_lifecycle,
        rviz_node,
        init_pose
    ])
