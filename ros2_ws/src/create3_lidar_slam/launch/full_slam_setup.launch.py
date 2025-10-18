# -------------------------------
# Create 3 LIDAR SLAM - Full Setup
# Launches:
#  1. RPLIDAR driver + TF publisher
#  2. SLAM Toolbox (for mapping/localization)
#  3. RViz2 for visualization
# -------------------------------

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Get path to this package ---
    pkg_dir = get_package_share_directory('create3_lidar_slam')

    # --- Include the lidar + TF launch file ---
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'sensors_launch.py'])
        )
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'ekf_launch.py'])
        )
    )

    # --- Include the SLAM Toolbox launch file ---
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'slam_toolbox_launch.py'])
        )
    )

    # --- Include the RViz visualization launch file ---
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'rviz_launch.py'])
        )
    )

    # --- Launch all three together ---
    return LaunchDescription([
        sensors_launch,
        ekf_launch,
        slam_toolbox_launch,
        rviz_launch
    ])