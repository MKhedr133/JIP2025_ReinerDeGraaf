#!/usr/bin/env python3
"""
Initial Pose Publisher

Publishes a single /initialpose after the /map topic (latched OccupancyGrid) is available.
This replaces the manual "2D Pose Estimate" click in RViz.

Params:
  initial_pose_x (float)        default: 0.0    # meters in map frame
  initial_pose_y (float)        default: 0.0    # meters in map frame
  initial_yaw_deg (float)       default: 0.0    # degrees (heading in map frame)
  frame_id (str)                default: "map"
  wait_for_map_timeout_sec      default: 30.0   # seconds to wait for /map before publishing anyway
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # ---------------- Parameters ----------------
        p = self.declare_parameter
        self.x = float(p('initial_pose_x', 0.0).value)
        self.y = float(p('initial_pose_y', 0.0).value)
        self.yaw_deg = float(p('initial_yaw_deg', 0.0).value)
        self.frame_id = str(p('frame_id', 'map').value)
        self.timeout = float(p('wait_for_map_timeout_sec', 30.0).value)

        # ---------------- Map wait (latched) ----------------
        # Subscribe to /map with TRANSIENT_LOCAL so we receive the latched map even if it was published earlier.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._map_evt = threading.Event()
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # ---------------- One-shot startup timer (keep handle so we can cancel) ----------------
        self._startup_timer = self.create_timer(0.1, self._start_once)

        self.get_logger().info(
            f"[InitPose] Waiting for /map (≤ {self.timeout:.0f}s). Will publish "
            f"({self.x:.2f}, {self.y:.2f}, {self.yaw_deg:.1f}°) in '{self.frame_id}'."
        )

    # Map callback: set the event once we see any map
    def _on_map(self, _msg: OccupancyGrid):
        if not self._map_evt.is_set():
            self._map_evt.set()

    # Timer callback: run once, after which we cancel the timer
    def _start_once(self):
        # Cancel this periodic timer; we only need it once
        self._startup_timer.cancel()

        # Wait (up to timeout) for /map to arrive
        got_map = self._map_evt.wait(timeout=self.timeout)
        if not got_map:
            self.get_logger().warn('[InitPose] Timeout waiting for /map; publishing initial pose anyway.')

        # Publish the initial pose one time
        self._publish_initial_pose()

    def _publish_initial_pose(self):
        yaw_rad = math.radians(self.yaw_deg)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id

        # Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        # Orientation (z-w for planar yaw)
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        # Light covariance (tune if needed)
        #  x,  y,  z,  rx, ry, rz (row-major 6x6)
        msg.pose.covariance = [
            0.25, 0,    0,    0,    0,    0,
            0,    0.25, 0,    0,    0,    0,
            0,    0,    0.10, 0,    0,    0,
            0,    0,    0,    0.10, 0,    0,
            0,    0,    0,    0,    0.10, 0,
            0,    0,    0,    0,    0,    0.10
        ]

        self.pub.publish(msg)
        self.get_logger().info(
            f"[InitPose] Published /initialpose at ({self.x:.2f}, {self.y:.2f}) yaw={self.yaw_deg:.1f}° in '{self.frame_id}'."
        )


def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
