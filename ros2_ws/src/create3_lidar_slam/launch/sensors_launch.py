from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ---- Launch-time args (simple, editable) ----
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0",
        description="RPLIDAR serial device"
    )
    serial_baud_arg = DeclareLaunchArgument(
        "serial_baudrate", default_value="115200",
        description="RPLIDAR serial baudrate"
    )
    frame_id_arg = DeclareLaunchArgument(
        "laser_frame", default_value="laser_frame",
        description="Frame id of the lidar"
    )
    # Offsets of lidar w.r.t. base_link (meters, radians)
    x_arg = DeclareLaunchArgument("x", default_value="-0.012")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.144")
    roll_arg = DeclareLaunchArgument("roll", default_value="0.0")
    pitch_arg = DeclareLaunchArgument("pitch", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    laser_frame = LaunchConfiguration("laser_frame")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    rr = LaunchConfiguration("roll")
    pp = LaunchConfiguration("pitch")
    yy = LaunchConfiguration("yaw")

    # ---- Static TF: base_link -> laser_frame ----
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        arguments=[x, y, z, rr, pp, yy, "base_link", laser_frame],
        output="screen",
    )

    # ---- RPLIDAR driver (composition) ----
    # We keep config in YAML but override port/baud/frame via params for clarity.
    rplidar = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_composition",
        parameters=[{
            "frame_id": laser_frame,
            "channel_type": "serial",
            "serial_port": serial_port,
            "serial_baudrate": serial_baudrate,
            "scan_mode": "",            # auto
            "angle_compensate": True,
            "auto_standby": True,
            "topic_name": "scan",
            "use_sim_time": False
        }],
        output="screen",
    )

    # Some drivers like a tiny delay after TF to avoid early TF lookup warnings
    delayed_lidar = TimerAction(period=2.0, actions=[rplidar])

    return LaunchDescription([
        serial_port_arg, serial_baud_arg, frame_id_arg,
        x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        static_tf,
        delayed_lidar,
    ])
