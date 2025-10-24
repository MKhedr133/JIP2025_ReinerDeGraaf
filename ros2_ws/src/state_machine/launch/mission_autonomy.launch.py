# Runs:
#   1) full_localization_setup.launch.py (map_server + AMCL + sensors + EKF)
#   2) nav2_bringup.launch.py (planner, controller, BT)
#   3) autonomous_controller.py (your radiation logic)
#   4) mission_manager (patrol + CPS trigger)
#
# The map path is computed from the installed share dir of create3_localization_bringup.
# The same path is fed to localization (map arg) and mission_manager (map_yaml param).

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    loc_share = get_package_share_directory('create3_localization_bringup')
    nav2_share = get_package_share_directory('create3_nav2_bringup')
    sm_share = get_package_share_directory('state_machine')

    default_map = os.path.join(loc_share, 'config', 'maps', 'create3_home_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Map YAML used by both localization and mission manager'
    )
    map_path = LaunchConfiguration('map')

    # 1) Localization (waits are handled inside mission_manager)
    full_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(loc_share, 'launch', 'full_localization_setup.launch.py')),
        launch_arguments={'map': map_path}.items()
    )

    # 2) Nav2 stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'nav2_bringup.launch.py'))
    )

    # 3) Autonomous radiation controller
    autonomous_controller = Node(
        package='main_controller',
        executable='autonomous_controller.py',
        name='autonomous_controller',
        output='screen'
    )

    # 4) Mission manager with config + map override
    mission_manager = Node(
        package='state_machine',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            os.path.join(sm_share, 'params', 'state_machine.yaml'),
            {'map_yaml': map_path}  # ensure generator uses the exact same map as AMCL
        ]
    )

    return LaunchDescription([
        map_arg,
        full_loc,
        nav2,
        autonomous_controller,
        mission_manager
    ])
