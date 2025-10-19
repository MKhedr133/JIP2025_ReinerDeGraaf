from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('create3_nav2_bringup'),
            'config', 'nav2_params.yaml'
        ),
        description='Full path to Nav2 parameters file'
    )
    params = LaunchConfiguration('params_file')

    planner = Node(package='nav2_planner', executable='planner_server',
                   name='planner_server', output='screen', parameters=[params])
    controller = Node(package='nav2_controller', executable='controller_server',
                      name='controller_server', output='screen', parameters=[params])
    smoother = Node(package='nav2_smoother', executable='smoother_server',
                    name='smoother_server', output='screen', parameters=[params])
    behavior = Node(package='nav2_behaviors', executable='behavior_server',
                    name='behavior_server', output='screen', parameters=[params])
    bt_nav = Node(package='nav2_bt_navigator', executable='bt_navigator',
                  name='bt_navigator', output='screen', parameters=[params])
    waypoint = Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                    name='waypoint_follower', output='screen', parameters=[params])

    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': True,
                     'node_names': [
                         'controller_server',
                         'planner_server',
                         'bt_navigator',
                         'behavior_server',
                         'smoother_server',
                         'waypoint_follower'
                     ]}]
    )

    return LaunchDescription([params_arg,
                              planner, controller, smoother,
                              behavior, bt_nav, waypoint, lifecycle])
