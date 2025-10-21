from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Robust path resolution
    pkg_share = get_package_share_directory("create3_lidar_slam")
    params_file = os.path.join(pkg_share, "config", "ekf_odom.yaml")

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[params_file]
    )

    return LaunchDescription([ekf])
