from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the keyboard listener node
        Node(
            package='keyboard_listener',
            executable='keyboard_listener',
            name='keyboard_listener',
            output='screen'
        ),
        # Launch the main controller node
        Node(
            package='main_controller',
            executable='main_controller',
            name='main_controller',
            output='screen'
        ),
        Node(
            package='drive_controller',
            executable='drive_control_node',
            name='drive_controller',
            output='screen'
        ),
        # # Launch scintillator nodes
        # Node(
        #     package='scintillator_lb124',
        #     executable='scintillator_raw_node',
        #     name='scintillator_raw',
        #     parameters=[{
        #         'port': '/dev/ttyUSB0',
        #         'baud': 19200,
        #         'timeout_s': 1.0,
        #     }],
        #     output='screen',
        # ),
        # Node(
        #     package='scintillator_lb124',
        #     executable='scintillator_filtered_node',
        #     name='scintillator_filter',
        #     parameters=[{
        #         'threshold': 20.0,
        #         'cps_index': 1,
        #     }], output='screen',
        # ), 
        # Node(
        #     package='scintillator_lb124',
        #     executable='scintillator_csv_node',
        #     name='scintillator_csv',
        #     parameters=[{
        #         'output_dir': '/home/user/scintillator_data',
        #         'add_header': True,
        #     }],
        #     output='screen',
        # ),
    ])