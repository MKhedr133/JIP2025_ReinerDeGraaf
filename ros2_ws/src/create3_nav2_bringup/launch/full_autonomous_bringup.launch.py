from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Package directories ---
    pkg_nav2 = get_package_share_directory('create3_nav2_bringup')
    pkg_localization = get_package_share_directory('create3_localization_bringup')
    pkg_slam = get_package_share_directory('create3_lidar_slam')

    # --- Config paths ---
    map_yaml = os.path.join(pkg_localization, 'config', 'maps', 'create3_map.yaml')
    ekf_yaml = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    nav2_params = os.path.join(pkg_nav2, 'config', 'nav2_params.yaml')

    # --- Launch arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use sim time'
    )

    # --- Nodes / Launch Includes ---

    ## 1. Static TF base_link → laser_frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['-0.012', '0.0', '0.144', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    ## 2. RPLIDAR driver
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{'serial_port': '/dev/rplidar', 'frame_id': 'laser_frame'}],
    )

    ## 3. EKF localization fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_yaml],
    )

    ## 4. Localization stack (map_server + AMCL)
    localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'amcl_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ## 5. Navigation (Nav2 full stack, no RViz)
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_setup.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ## 6. Optional initial pose (auto-inject)
    init_pose = Node(
        package='create3_nav2_bringup',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen'
    )

    # --- Optional: RViz launch toggle (comment out for debugging) ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_nav2, 'config', 'nav2_default_view.rviz')]
    )

    # --- Return full description ---
    return LaunchDescription([
        declare_use_sim_time,
        static_tf,
        lidar_node,
        ekf_node,
        localization_include,
        nav2_include,
        init_pose,
        rviz_node  # enable later once verified
    ])
