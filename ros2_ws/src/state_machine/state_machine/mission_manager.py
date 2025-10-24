# Mission flow:
#   - Wait for /map (latched) and Nav2 action servers to be ready
#   - Build a patrol route (generated/yaml/list)
#   - Navigate poses (sequential or batch)
#   - On CPS >= threshold: cancel Nav2, publish 'DRIVE' to /simba_state
#   - On 'STOP' from your controller: optionally resume patrol

import math
import threading
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid

from .utils import yaw_to_quat
from .route_generators import boustrophedon_sweep, load_waypoint_list_from_yaml

PATROL = 'PATROL'
SWEEP  = 'SWEEP'
PAUSED = 'PAUSED'

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # ---------------- Parameters ----------------
        p = self.declare_parameter
        self.cps_topic     = p('cps_topic', '/scintillator/cps').value
        self.cps_trigger   = float(p('cps_trigger', 20.0).value)
        self.frame_id      = p('frame_id', 'map').value

        self.route_source  = p('route_source', 'generated').value   # generated|yaml|list
        self.map_yaml      = p('map_yaml', '/dev/null').value
        self.lane_spacing  = float(p('lane_spacing_m', 1.0).value)
        self.heading_deg   = float(p('heading_deg', 0.0).value)
        self.inline_wps    = p('waypoints', []).value

        self.nav_mode      = p('nav_mode', 'sequential').value      # sequential|batch
        self.resume_after  = bool(p('resume_patrol_after_sweep', True).value)

        self.wait_for_map          = bool(p('wait_for_map', True).value)
        self.wait_for_map_timeout  = float(p('wait_for_map_timeout_sec', 30.0).value)
        self.wait_for_nav2_timeout = float(p('wait_for_nav2_timeout_sec', 60.0).value)

        # ---------------- IO ----------------
        # CPS
        self.create_subscription(Float32, self.cps_topic, self._on_cps, 10)
        # State handoff
        self.state_pub = self.create_publisher(String, '/simba_state', 10)
        self.create_subscription(String, '/simba_state', self._on_simba_state, 10)

        # Map (latched)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_event = threading.Event()
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        # Actions
        self.nav_to_pose_client   = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nav_through_client   = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.current_goal_handle  = None

        # State
        self.mode          = PATROL
        self.patrol: List[Dict] = []
        self.patrol_index  = 0
        self.triggered     = False

        # Kick off the startup sequence
        self.create_timer(0.1, self._startup_once)

        self.get_logger().info('[Mission] Node up. Waiting for map/Nav2 before generating waypoints.')

    # ---------------- Startup wait sequence ----------------
    def _startup_once(self):
        # run only once
        self._startup_once.cancel()

        # 1) Wait for /map
        if self.wait_for_map:
            self.get_logger().info(f'[Mission] Waiting for /map (≤ {self.wait_for_map_timeout:.0f}s)...')
            got_map = self.map_event.wait(timeout=self.wait_for_map_timeout)
            if not got_map:
                self.get_logger().warn('[Mission] Timeout waiting for /map. Continuing anyway.')
        else:
            self.get_logger().info('[Mission] Skipping wait for /map.')

        # 2) Build patrol (after we know map is ready and map_yaml was overridden by launch)
        self.patrol = self._build_patrol()
        self.get_logger().info(f'[Mission] Patrol has {len(self.patrol)} waypoints.')

        # 3) Wait for Nav2 action server(s)
        if self.nav_mode == 'batch':
            self._wait_for_action(self.nav_through_client, '/navigate_through_poses', self.wait_for_nav2_timeout)
            self._send_batch()
        else:
            self._wait_for_action(self.nav_to_pose_client, '/navigate_to_pose', self.wait_for_nav2_timeout)
            self._send_next()

    def _on_map(self, _msg: OccupancyGrid):
        # Just receiving once is enough to know map_server is alive and RViz can render.
        if not self.map_event.is_set():
            self.map_event.set()

    def _wait_for_action(self, client: ActionClient, name: str, timeout: float):
        self.get_logger().info(f'[Mission] Waiting for {name} (≤ {timeout:.0f}s)...')
        ok = client.wait_for_server(timeout_sec=timeout)
        if not ok:
            self.get_logger().warn(f'[Mission] Timeout waiting for {name}; will try to send anyway.')

    # ---------------- Patrol building ----------------
    def _build_patrol(self) -> List[Dict]:
        if self.route_source == 'generated':
            self.get_logger().info(f'[Mission] Generating waypoints from map: {self.map_yaml}')
            return boustrophedon_sweep(self.map_yaml, lane_spacing_m=self.lane_spacing, heading_deg=self.heading_deg)
        elif self.route_source == 'yaml':
            self.get_logger().info(f'[Mission] Loading waypoints list YAML: {self.map_yaml}')
            return load_waypoint_list_from_yaml(self.map_yaml)
        else:  # 'list'
            self.get_logger().info('[Mission] Using inline waypoints from params.')
            return list(self.inline_wps)

    # ---------------- Nav: sequential ----------------
    def _send_next(self):
        if self.mode != PATROL or not self.patrol:
            if not self.patrol:
                self.get_logger().error('[Mission] No patrol waypoints.')
            return
        if self.patrol_index >= len(self.patrol):
            self.patrol_index = 0
        wp = self.patrol[self.patrol_index]
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(wp['x'], wp['y'], wp.get('theta_deg', 0.0))
        self.get_logger().info(f"[Mission] → Goal {self.patrol_index+1}/{len(self.patrol)}: "
                               f"({wp['x']:.2f},{wp['y']:.2f}) θ={wp.get('theta_deg',0):.1f}°")
        fut = self.nav_to_pose_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('[Mission] Goal rejected; advancing.')
            self.patrol_index += 1
            self._send_next()
            return
        self.current_goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, fut):
        if self.mode != PATROL:
            return
        self.get_logger().info('[Mission] Goal finished.')
        self.patrol_index += 1
        self._send_next()

    # ---------------- Nav: batch ----------------
    def _send_batch(self):
        if self.mode != PATROL or not self.patrol:
            if not self.patrol:
                self.get_logger().error('[Mission] No patrol waypoints.')
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._make_pose(w['x'], w['y'], w.get('theta_deg', 0.0)) for w in self.patrol]
        self.get_logger().info(f'[Mission] → Batch of {len(goal.poses)} poses.')
        fut = self.nav_through_client.send_goal_async(goal)
        fut.add_done_callback(self._batch_goal_response_cb)

    def _batch_goal_response_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('[Mission] Batch goal rejected.')
            return
        self.current_goal_handle = gh
        gh.get_result_async().add_done_callback(self._batch_result_cb)

    def _batch_result_cb(self, _fut):
        if self.mode != PATROL:
            return
        self.get_logger().info('[Mission] Batch finished; looping.')
        self._send_batch()

    # ---------------- CPS trigger & handoff ----------------
    def _on_cps(self, msg: Float32):
        if self.mode != PATROL:
            return
        if msg.data >= self.cps_trigger:
            self.get_logger().warn(f'[Mission] CPS {msg.data:.1f} ≥ {self.cps_trigger} → HANDOFF to radiation controller.')
            self.mode = SWEEP
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
            self.state_pub.publish(String(data='DRIVE'))

    def _on_simba_state(self, msg: String):
        if self.mode == SWEEP and msg.data.strip().upper() == 'STOP':
            self.get_logger().info('[Mission] Radiation sweep finished.')
            if self.resume_after:
                self.mode = PATROL
                self.current_goal_handle = None
                if self.nav_mode == 'batch':
                    self._send_batch()
                else:
                    self._send_next()
            else:
                self.mode = PAUSED

    # ---------------- Helpers ----------------
    def _make_pose(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = Time()  # frame is what Nav2 cares about
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = yaw_to_quat(math.radians(float(yaw_deg)))
        return ps

def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
