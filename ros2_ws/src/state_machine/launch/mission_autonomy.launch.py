from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    loc_share = get_package_share_directory('create3_localization_bringup')
    nav2_pkg = get_package_share_directory('create3_nav2_bringup')
    sm_pkg   = get_package_share_directory('state_machine')

    default_map = os.path.join(loc_share, 'config', 'maps', 'create3_home_map.yaml')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to the map YAML used by both localization and mission manager'
    )

    map_path = LaunchConfiguration('map')

    # 1) Localization (your existing full stack with AMCL + map server + sensors + EKF)
    full_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'full_localization_setup.launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )

    # 2) Nav2 bringup (planner, controller, bt_navigator, waypoint follower optional)
    nav2 = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(nav2_pkg, 'launch', 'nav2_bringup.launch.py')))

    # 3) Your autonomous radiation controller
    autonomous_controller = Node(
        package='main_controller',
        executable='autonomous_controller.py',  # if installed as script use 'autonomous_controller'
        name='autonomous_controller',
        output='screen'
    )

    # 4) Mission manager with params file (can override on CLI)
    mission_manager = Node(
        package='state_machine',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            os.path.join(sm_share, 'params', 'state_machine.yaml'),
            {'map_yaml': map_path}   # <<< critical: force same map for generator
        ]
    )

    return LaunchDescription([
        map_arg,
        full_loc,
        nav2,
        autonomous_controller,
        mission_manager
    ])

