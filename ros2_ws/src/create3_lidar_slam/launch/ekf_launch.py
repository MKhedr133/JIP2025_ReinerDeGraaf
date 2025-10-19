from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic", default_value="/imu",
        description="IMU topic (e.g., /imu or /imu/data)"
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic", default_value="/odom",
        description="Wheel odometry topic"
    )

    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    # Robust path resolution
    pkg_share = get_package_share_directory("create3_lidar_slam")
    params_file = os.path.join(pkg_share, "config", "ekf_odom.yaml")

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("/odom", odom_topic),
            ("/imu", imu_topic),
        ],
    )

    return LaunchDescription([imu_topic_arg, odom_topic_arg, ekf])
