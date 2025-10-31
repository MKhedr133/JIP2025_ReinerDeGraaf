from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    loc_share = get_package_share_directory('create3_localization_bringup')
    nav2_share = get_package_share_directory('create3_nav2_bringup')
    sm_share   = get_package_share_directory('state_machine')

    default_map = os.path.join(loc_share, 'config', 'maps', 'create3_home_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Map YAML for both localization and mission_manager'
    )
    map_path = LaunchConfiguration('map')

    full_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'full_localization_setup.launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'nav2_bringup.launch.py')
        )
    )

    mission_manager = Node(
        package='state_machine',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            os.path.join(sm_share, 'params', 'state_machine.yaml'),
            {
                'map_yaml': map_path,
                # Nav2-only mode:
                'sweep_enabled': False,                 # <— ignore CPS triggers
                'resume_patrol_after_sweep': True,      # harmless here
                # keep these so we only start after you localize:
                'require_initialpose_click': True,
                'amcl_convergence_check': False,
                'heading_mode': 'align_to_initial_yaw',
                'snap_heading_to_90deg': True,
            }
        ]
    )

    return LaunchDescription([map_arg, full_loc, nav2, mission_manager])
