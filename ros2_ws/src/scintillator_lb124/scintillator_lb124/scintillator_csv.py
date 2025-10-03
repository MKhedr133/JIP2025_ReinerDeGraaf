#!/usr/bin/env python3
"""
CPS → CSV logger (default saved inside the package)
==================================================
Subscribes to `/scintillator/cps` and writes `time,cps` to a CSV file.
By default the file is placed in the installed package share directory under `csv/`.

Parameters
- output_dir (string, default: <pkg_share>/csv)
- file_prefix (string, default: "scint_log_")
- add_header (bool, default: true)
"""

import os, csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from irobot_create_msgs.msg import WheelTicks
from ament_index_python.packages import get_package_share_directory
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class ScintillatorCsvNode(Node):
    def __init__(self):
        super().__init__("scintillator_csv")
        # Resolve <pkg_share>/csv as the default output directory
        default_dir = os.path.join(get_package_share_directory("scintillator_lb124"), "csv")
        os.makedirs(default_dir, exist_ok=True)

        self.declare_parameter("output_dir", default_dir)
        self.declare_parameter("file_prefix", "scint_log_")
        self.declare_parameter("add_header", True)

        out_dir = str(self.get_parameter("output_dir").value)
        prefix = str(self.get_parameter("file_prefix").value)
        add_header = bool(self.get_parameter("add_header").value)

        fname = f"{prefix}{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.path = os.path.join(out_dir, fname)
        self.fh = open(self.path, "w", newline="")
        self.writer = csv.writer(self.fh)

        if add_header:
            self.writer.writerow(["time", "cps", "distance_cm"])
            self.fh.flush()

        # Initialize latest data
        self.latest_cps = None
        self.latest_left_ticks = None
        self.latest_right_ticks = None
        self.start_left_ticks = None
        self.start_right_ticks = None
        self.circumference = math.pi * 72.0  # in mm

        # Control logging via this flag
        self.should_log = False

        # make sure QOS lines up with roomba stuff
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.create_subscription(Float32, "/scintillator/cps", self.on_cps, 10)
        self.create_subscription(WheelTicks, "wheel_ticks", self.on_wheel_tick, qos)
        self.create_subscription(String, "/simba_state", self.on_simba_state, 10)


        # Timer: log data x times per second
        self.create_timer(0.1, self.log_data)

        self.get_logger().info(f"Logging CPS to {self.path}")

    def on_simba_state(self, msg: String):
        if msg.data.strip().upper() == "EXPERIMENT":
            if not self.should_log:
                self.get_logger().info("Received 'EXPERIMENT' on /simba_state → Starting CSV logging")
            self.should_log = True
            self.start_left_ticks = self.latest_left_ticks
            self.start_right_ticks = self.latest_right_ticks
        else:
            if self.should_log:
                self.get_logger().info(f"/simba_state changed to '{msg.data}' → Stopping CSV logging")
            self.should_log = False


    def on_cps(self, msg: Float32):
        self.latest_cps = msg.data

    def on_wheel_tick(self, msg: WheelTicks):
        self.latest_left_ticks = msg.ticks_left
        self.latest_right_ticks = msg.ticks_right

    def log_data(self):
        if not self.should_log:
            return  
        

        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        cps = f"{self.latest_cps:.2f}" if self.latest_cps is not None else ""

        if self.start_left_ticks is not None or self.start_right_ticks is not None:
            delta_ticks = ((self.latest_left_ticks - self.start_left_ticks) + (self.latest_right_ticks - self.start_right_ticks)) / 2.0 
            distance_cm = ((delta_ticks / 508.8) * self.circumference)/ 100.0 if self.latest_left_ticks is not None and self.latest_right_ticks is not None else ""
        else:
            distance_cm = None
        self.writer.writerow([now, cps, distance_cm])
        self.fh.flush()
        # self.get_logger().info("Logged data row")
        self.get_logger().info(f"Logged cps: {cps}, dist: {distance_cm} cm")

    def destroy_node(self):
        try:
            self.fh.flush(); self.fh.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ScintillatorCsvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()