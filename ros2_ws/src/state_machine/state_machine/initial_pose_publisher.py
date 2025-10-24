# Publishes a single /initialpose after /map is available (latched).
# Parameters:
#   initial_pose_x, initial_pose_y, initial_yaw_deg, frame_id
#   wait_for_map_timeout_sec

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        p = self.declare_parameter
        self.x   = float(p('initial_pose_x', 0.0).value)
        self.y   = float(p('initial_pose_y', 0.0).value)
        self.yaw = math.radians(float(p('initial_yaw_deg', 0.0).value))
        self.frame_id = p('frame_id', 'map').value
        self.timeout  = float(p('wait_for_map_timeout_sec', 30.0).value)

        # Wait for /map (latched)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._map_evt = threading.Event()
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(0.1, self._start_once)

    def _on_map(self, _msg):
        self._map_evt.set()

    def _start_once(self):
        self._start_once.cancel()
        self.get_logger().info(f'[InitPose] Waiting for /map (≤ {self.timeout:.0f}s)...')
        got = self._map_evt.wait(timeout=self.timeout)
        if not got:
            self.get_logger().warn('[InitPose] Timeout waiting for /map; publishing anyway.')
        self._publish()

    def _publish(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw/2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw/2.0)
        # light covariance
        msg.pose.covariance = [0.25,0,0,0,0,0,
                               0,0.25,0,0,0,0,
                               0,0,0.1,0,0,0,
                               0,0,0,0.1,0,0,
                               0,0,0,0,0.1,0,
                               0,0,0,0,0,0.1]
        self.pub.publish(msg)
        self.get_logger().info(f'[InitPose] Published /initialpose at ({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.1f}°')

def main():
    rclpy.init()
    rclpy.spin(InitialPosePublisher())
    rclpy.shutdown()
