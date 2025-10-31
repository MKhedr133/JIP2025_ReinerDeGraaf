from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    localization_pkg = get_package_share_directory("create3_localization_bringup")
    lidar_pkg = get_package_share_directory("create3_lidar_slam")

    # Arguments
    mode_arg = DeclareLaunchArgument("mode", default_value="auto", description="fixed | auto")
    amcl_ns_arg = DeclareLaunchArgument("amcl_namespace", default_value="", description="Namespace of AMCL node if any")
    x_arg = DeclareLaunchArgument("initial_pose_x", default_value="0.0")
    y_arg = DeclareLaunchArgument("initial_pose_y", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("initial_yaw_deg", default_value="0.0")
    do_sweep_arg = DeclareLaunchArgument("do_yaw_sweep", default_value="true")

    # Config paths
    map_yaml = os.path.join(localization_pkg, "config", "maps", "create3_home_map.yaml")
    amcl_yaml = os.path.join(localization_pkg, "config", "amcl_params.yaml")
    ekf_yaml  = os.path.join(get_package_share_directory("create3_lidar_slam"), "config", "ekf_odom.yaml")

    # Sensors & EKF (reuse your existing)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, "launch", "sensors_launch.py")
        )
    )
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, "launch", "ekf_launch.py")
        )
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_yaml],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"]
        }],
    )

    # Helper: auto-initialize pose
    auto_init = Node(
        package="create3_localization_bringup",
        executable="auto_initial_pose",
        name="auto_initial_pose",
        output="screen",
        parameters=[{
            "mode": LaunchConfiguration("mode"),
            "amcl_namespace": LaunchConfiguration("amcl_namespace"),
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_yaw_deg": LaunchConfiguration("initial_yaw_deg"),
            "do_yaw_sweep": LaunchConfiguration("do_yaw_sweep"),
            # optional thresholds/timeouts use defaults from script
        }],
    )

    # Delay helper until AMCL is active
    delayed_auto = TimerAction(period=3.0, actions=[auto_init])

    return LaunchDescription([
        mode_arg, amcl_ns_arg, x_arg, y_arg, yaw_arg, do_sweep_arg,
        sensors_launch,
        ekf_launch,
        map_server,
        amcl,
        lifecycle_manager,
        delayed_auto
    ])
