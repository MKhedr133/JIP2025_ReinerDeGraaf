# Launches:
#   1) create3_localization_bringup/launch/full_localization_setup.launch.py (no auto 2D pose)
#   2) create3_nav2_bringup/launch/nav2_bringup.launch.py
#   3) main_controller (keyboard/supervisor)  → remapped to /simba_state
#   4) ONE search strategy from main_controller (basic/gradient/double_sweep/gradient_double_sweep)
#   5) state_machine/mission_manager (Nav2 patrol ↔ search handoff)
#
# Map path is resolved from the installed create3_localization_bringup share and injected
# into BOTH localization and mission_manager (as map/map_yaml).

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Installed share directories
    loc_share = get_package_share_directory('create3_localization_bringup')
    nav2_share = get_package_share_directory('create3_nav2_bringup')
    sm_share   = get_package_share_directory('state_machine')
    mc_share   = get_package_share_directory('main_controller')

    # Default map (installed)
    default_map = os.path.join(loc_share, 'config', 'maps', 'create3_home_map.yaml')

    # --- Launch args ---
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to the map YAML used by both localization and mission manager'
    )

    strategy_arg = DeclareLaunchArgument(
        'strategy',
        default_value='basic',
        description='Search strategy: {basic, gradient, double_sweep, gradient_double_sweep}'
    )

    strategy_params_arg = DeclareLaunchArgument(
        'strategy_params',
        default_value=os.path.join(mc_share, 'config', 'autonomy_params.yaml'),
        description='YAML params for the selected autonomous controller'
    )

    map_path = LaunchConfiguration('map')
    strategy = LaunchConfiguration('strategy')
    strategy_params = LaunchConfiguration('strategy_params')

    # 1) Localization
    full_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_share, 'launch', 'full_localization_setup.launch.py')
        ),
        launch_arguments={'map': map_path}.items()
    )

    # 2) Nav2 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'nav2_bringup.launch.py')
        )
    )

    # 3) Main controller
    #    main_controller publishes `simba_state` (relative). Remap → `/simba_state` for consistency.
    main_controller_node = Node(
        package='main_controller',
        executable='main_controller',   # entry point from setup.py
        name='main_controller',
        output='screen',
        remappings=[('simba_state', '/simba_state')]
    )

    # 4) One autonomous search strategy (select via --ros-args strategy:=...)
    strat_basic = Node(
        condition=IfCondition(PythonExpression(["'", strategy, "' == 'basic'"])),
        package='main_controller',
        executable='basic',
        name='autonomy_basic',
        output='screen',
        parameters=[strategy_params],
        remappings=[('simba_state', '/simba_state')]
    )

    strat_gradient = Node(
        condition=IfCondition(PythonExpression(["'", strategy, "' == 'gradient'"])),
        package='main_controller',
        executable='gradient',
        name='autonomy_gradient',
        output='screen',
        parameters=[strategy_params],
        remappings=[('simba_state', '/simba_state')]
    )

    strat_double = Node(
        condition=IfCondition(PythonExpression(["'", strategy, "' == 'double_sweep'"])),
        package='main_controller',
        executable='double_sweep',
        name='autonomy_double_sweep',
        output='screen',
        parameters=[strategy_params],
        remappings=[('simba_state', '/simba_state')]
    )

    strat_grad_double = Node(
        condition=IfCondition(PythonExpression(["'", strategy, "' == 'gradient_double_sweep'"])),
        package='main_controller',
        executable='gradient_double_sweep',
        name='autonomy_gradient_double_sweep',
        output='screen',
        parameters=[strategy_params],
        remappings=[('simba_state', '/simba_state')]
    )

    # 5) Mission manager (uses same map via map_yaml override)
    mission_manager = Node(
        package='state_machine',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            os.path.join(sm_share, 'params', 'state_machine.yaml'),
            {'map_yaml': map_path}
        ]
    )

    return LaunchDescription([
        map_arg, strategy_arg, strategy_params_arg,
        full_loc,
        nav2,
        main_controller_node,
        strat_basic, strat_gradient, strat_double, strat_grad_double,
        mission_manager
    ])
