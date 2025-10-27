from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="simba_bringup",
            executable="servo_controller",
            name="servo_controller",
            output="screen",
            parameters=[{
                "gpio_pin": 18,          # BCM 18 (hardware PWM)
                "min_pulse_us": 1500,    # per your spec (CCW 1500→2500)
                "max_pulse_us": 2500,
                "min_angle_deg": 0.0,
                "max_angle_deg": 90.0,
                "startup_angle_deg": 45.0
            }]
        )
    ])
