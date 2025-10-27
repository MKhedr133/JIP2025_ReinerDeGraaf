from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    param_file = PathJoinSubstitution([
        FindPackageShare('main_controller'),
        'config',
        'autonomy_params.yaml'
    ])    
    # Declare a launch argument for the autonomy mode
    autonomy_mode_arg = DeclareLaunchArgument(
        'autonomy_mode', default_value='basic',
        description='Which autonomous controller to run. Options: basic, gradient, double_sweep, gradient_double_sweep'
    )

    return LaunchDescription([
        # Add the launch argument to the launch description
        autonomy_mode_arg,
        # Launch the main controller node
        Node(
            package='main_controller',
            executable='main_controller',
            name='main_controller',
            output='screen'
        ),
        # Launch the autonomous controller node
        Node(
            package='main_controller',
            executable=LaunchConfiguration('autonomy_mode'),
            name='autonomous_controller',
            output='screen',
            parameters=[param_file]
        )  
    ])