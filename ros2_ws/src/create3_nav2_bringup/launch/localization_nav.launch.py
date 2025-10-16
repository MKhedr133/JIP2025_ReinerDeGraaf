from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'create3_nav2_bringup'
    pkg_share = get_package_share_directory(pkg_name)

    # -------------------
    # Launch arguments
    # -------------------
    map_yaml = DeclareLaunchArgument(
        'map',
        default_value='/home/ubuntu/maps/create3_map.yaml',
        description='Full path to map YAML file'
    )

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Default behavior tree
    bt_xml_file = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    )

    # Ensure consistent locale for FastDDS and Nav2 logs
    env_vars = SetEnvironmentVariable('LC_NUMERIC', 'C')

    # -------------------
    # Nodes
    # -------------------
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': LaunchConfiguration('map')}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file,
                    {'default_nav_to_pose_bt_xml': bt_xml_file}]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[params_file]
    )

    # -------------------
    # Launch description
    # -------------------
    ld = LaunchDescription()

    ld.add_action(env_vars)
    ld.add_action(map_yaml)

    for n in [
        map_server, amcl, planner, controller, smoother,
        behavior_server, bt_nav, waypoint_follower, lifecycle_mgr
    ]:
        ld.add_action(n)

    return ld
