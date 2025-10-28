#!/usr/bin/env python3
# Nav2 patrol ↔ Search strategy handoff with:
# - /map wait, Nav2 wait
# - Optional wait for manual /initialpose + AMCL convergence
# - Sweep heading aligned to initial yaw (snapped to 90°)
# - CPS trigger → cancel Nav2, publish DRIVE; timeout → STOP and resume

import math
import threading
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.timer import Timer

from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid

from .utils import yaw_to_quat
from .route_generators import boustrophedon_sweep, load_waypoint_list_from_yaml

PATROL = 'PATROL'
SWEEP = 'SWEEP'
PAUSED = 'PAUSED'

def quat_to_yaw_deg(q) -> float:
    # generic quaternion -> yaw (deg)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        p = self.declare_parameter

        # === Core knobs ===
        self.cps_topic     = p('cps_topic', '/scintillator/cps').value
        self.cps_trigger   = float(p('cps_trigger', 20.0).value)
        self.frame_id      = p('frame_id', 'map').value
        self.sweep_enabled = bool(self.declare_parameter('sweep_enabled', True).value)

        self.route_source  = p('route_source', 'generated').value   # generated|yaml|list
        self.map_yaml      = p('map_yaml', '/dev/null').value
        self.lane_spacing  = float(p('lane_spacing_m', 1.0).value)
        self.heading_deg   = float(p('heading_deg', 0.0).value)     # used when heading_mode == fixed
        self.inline_wps    = p('waypoints', []).value

        self.nav_mode      = p('nav_mode', 'sequential').value      # sequential|batch
        self.resume_after  = bool(p('resume_patrol_after_sweep', True).value)

        self.wait_for_map          = bool(p('wait_for_map', True).value)
        self.wait_for_map_timeout  = float(p('wait_for_map_timeout_sec', 30.0).value)
        self.wait_for_nav2_timeout = float(p('wait_for_nav2_timeout_sec', 60.0).value)
        self.sweep_timeout_sec     = float(p('sweep_timeout_sec', 600.0).value)

        # === Gating on manual 2D Pose + AMCL convergence ===
        self.require_initialpose_click = bool(p('require_initialpose_click', True).value)
        self.initialpose_timeout_sec   = float(p('initialpose_timeout_sec', 120.0).value)

        self.check_amcl_convergence    = bool(p('amcl_convergence_check', True).value)
        self.amcl_wait_timeout_sec     = float(p('amcl_wait_timeout_sec', 60.0).value)
        self.amcl_cov_xy_max           = float(p('amcl_cov_xy_max', 0.05).value)      # m^2 (variance)
        self.amcl_cov_yaw_deg_max      = float(p('amcl_cov_yaw_deg_max', 5.0).value)  # deg (stddev, approx)

        # === How to set waypoint heading ===
        # 'fixed' → use heading_deg param
        # 'align_to_initial_yaw' → use yaw from /initialpose or /amcl_pose
        self.heading_mode              = p('heading_mode', 'align_to_initial_yaw').value
        self.snap_heading_to_90deg     = bool(p('snap_heading_to_90deg', True).value)

        # IO
        self.create_subscription(Float32, self.cps_topic, self._on_cps, 10)
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
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        # Wait for /initialpose and /amcl_pose
        self.initialpose_event = threading.Event()
        self.amcl_converged_event = threading.Event()
        self.initial_yaw_deg: Optional[float] = None

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)

        # Nav2 actions
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')

        # State
        self.mode = PATROL
        self.patrol: List[Dict] = []
        self.patrol_index = 0
        self.current_goal_handle = None
        self.sweep_timer: Optional[Timer] = None

        # Startup
        self._startup_timer = self.create_timer(0.1, self._startup_once)
        self.get_logger().info('[Mission] Up. Will wait for map/Nav2, and (optionally) initial pose + AMCL.')

    # ---------- Callbacks ----------
    def _on_map(self, _msg: OccupancyGrid):
        self.map_event.set()

    def _on_initialpose(self, msg: PoseWithCovarianceStamped):
        self.initial_yaw_deg = quat_to_yaw_deg(msg.pose.pose.orientation)
        self.initialpose_event.set()
        self.get_logger().info(f"[Mission] Got /initialpose. Yaw={self.initial_yaw_deg:.1f}°")

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance  # 6x6 row-major
        var_x = cov[0]
        var_y = cov[7]
        var_yaw = cov[35]            # variance around z
        std_yaw_deg = math.degrees(math.sqrt(max(0.0, var_yaw)))
        if var_x <= self.amcl_cov_xy_max and var_y <= self.amcl_cov_xy_max and std_yaw_deg <= self.amcl_cov_yaw_deg_max:
            if not self.amcl_converged_event.is_set():
                self.amcl_converged_event.set()
                self.get_logger().info(f"[Mission] AMCL converged (σ_yaw≈{std_yaw_deg:.1f}°).")

        # keep latest yaw as fallback if no /initialpose was seen
        if self.initial_yaw_deg is None:
            self.initial_yaw_deg = quat_to_yaw_deg(msg.pose.pose.orientation)

    # ---------- Startup waiting ----------
    def _startup_once(self):
        self._startup_timer.cancel()

        # 1) Wait for /map
        if self.wait_for_map:
            self.get_logger().info(f'[Mission] Waiting for /map (≤ {self.wait_for_map_timeout:.0f}s)...')
            if not self.map_event.wait(timeout=self.wait_for_map_timeout):
                self.get_logger().warn('[Mission] Timeout waiting for /map; continuing anyway.')

        # 2) Wait for manual 2D Pose Estimate (if required)
        if self.require_initialpose_click:
            self.get_logger().info(f'[Mission] Waiting for /initialpose click (≤ {self.initialpose_timeout_sec:.0f}s)...')
            if not self.initialpose_event.wait(timeout=self.initialpose_timeout_sec):
                self.get_logger().warn('[Mission] No /initialpose received before timeout; continuing without it.')

        # 3) Wait for AMCL convergence (optional)
        if self.check_amcl_convergence:
            self.get_logger().info(f'[Mission] Waiting for AMCL convergence (≤ {self.amcl_wait_timeout_sec:.0f}s)...')
            if not self.amcl_converged_event.wait(timeout=self.amcl_wait_timeout_sec):
                self.get_logger().warn('[Mission] AMCL did not meet convergence criteria in time; continuing.')

        # 4) Build patrol (with heading alignment if requested)
        heading_for_sweep = self._compute_sweep_heading_deg()
        self.get_logger().info(f'[Mission] Using sweep heading: {heading_for_sweep:.1f}°')

        self.patrol = self._build_patrol(heading_for_sweep)
        self.get_logger().info(f'[Mission] Patrol size: {len(self.patrol)} waypoints.')

        # 5) Wait for Nav2 action server(s) and start
        if self.nav_mode == 'batch':
            self._wait_for_action(self.nav_through_client, '/navigate_through_poses', self.wait_for_nav2_timeout)
            self._send_batch()
        else:
            self._wait_for_action(self.nav_to_pose_client, '/navigate_to_pose', self.wait_for_nav2_timeout)
            self._send_next()

    def _compute_sweep_heading_deg(self) -> float:
        if self.heading_mode != 'align_to_initial_yaw' or self.initial_yaw_deg is None:
            return float(self.heading_deg)
        yaw = self.initial_yaw_deg
        if self.snap_heading_to_90deg:
            yaw = round(yaw / 90.0) * 90.0
        # normalize to [-180, 180]
        yaw = (yaw + 180.0) % 360.0 - 180.0
        return yaw

    # ---------- Helpers ----------
    def _wait_for_action(self, client: ActionClient, name: str, timeout: float):
        self.get_logger().info(f'[Mission] Waiting for {name} (≤ {timeout:.0f}s)...')
        ok = client.wait_for_server(timeout_sec=timeout)
        if not ok:
            self.get_logger().warn(f'[Mission] Timeout waiting for {name}; attempting to proceed.')

    def _build_patrol(self, heading_for_sweep: float) -> List[Dict]:
        if self.route_source == 'generated':
            self.get_logger().info(f'[Mission] Generating waypoints from map: {self.map_yaml}')
            return boustrophedon_sweep(
                self.map_yaml,
                lane_spacing_m=self.lane_spacing,
                heading_deg=heading_for_sweep
            )
        elif self.route_source == 'yaml':
            self.get_logger().info(f'[Mission] Loading waypoint list from: {self.map_yaml}')
            wps = load_waypoint_list_from_yaml(self.map_yaml)
            # overwrite heading if align mode is active
            if self.heading_mode == 'align_to_initial_yaw':
                for w in wps:
                    w['theta_deg'] = heading_for_sweep
            return wps
        else:
            self.get_logger().info('[Mission] Using inline waypoint list from params.')
            wps = list(self.inline_wps)
            if self.heading_mode == 'align_to_initial_yaw':
                for w in wps:
                    w['theta_deg'] = heading_for_sweep
            return wps

    # ---------- Navigate (sequential) ----------
    def _send_next(self):
        if self.mode != PATROL or not self.patrol:
            if not self.patrol:
                self.get_logger().error('[Mission] No patrol waypoints.')
            return
        if self.patrol_index >= len(self.patrol):
            self.patrol_index = 0
        wp = self.patrol[self.patrol_index]
        goal = NavigateToPose.Goal()
        goal.pose = self._mk_pose(wp['x'], wp['y'], wp.get('theta_deg', 0.0))
        self.get_logger().info(f"[Mission] → Goal {self.patrol_index+1}/{len(self.patrol)}: "
                               f"({wp['x']:.2f},{wp['y']:.2f}) θ={wp.get('theta_deg',0):.1f}°")
        fut = self.nav_to_pose_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_resp_cb)

    def _goal_resp_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('[Mission] Goal rejected; advancing.')
            self.patrol_index += 1
            self._send_next()
            return
        self.current_goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, _fut):
        if self.mode != PATROL:
            return
        self.get_logger().info('[Mission] Goal finished.')
        self.patrol_index += 1
        self._send_next()

    # ---------- Navigate (batch) ----------
    def _send_batch(self):
        if self.mode != PATROL or not self.patrol:
            if not self.patrol:
                self.get_logger().error('[Mission] No patrol waypoints.')
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._mk_pose(w['x'], w['y'], w.get('theta_deg', 0.0)) for w in self.patrol]
        self.get_logger().info(f'[Mission] → Batch of {len(goal.poses)} poses.')
        fut = self.nav_through_client.send_goal_async(goal)
        fut.add_done_callback(self._batch_goal_resp_cb)

    def _batch_goal_resp_cb(self, fut):
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

    # ---------- CPS trigger & handoff ----------
    def _on_cps(self, msg: Float32):
        if not self.sweep_enabled:
            return
        if self.mode != PATROL:
            return
        if msg.data >= self.cps_trigger:
            self.get_logger().warn(f'[Mission] CPS {msg.data:.1f} ≥ {self.cps_trigger} → HANDOFF to search strategy.')
            self.mode = SWEEP
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            self.state_pub.publish(String(data='DRIVE'))
            if self.sweep_timeout_sec > 0:
                self.sweep_timer = self.create_timer(self.sweep_timeout_sec, self._sweep_timeout_once)

    def _sweep_timeout_once(self):
        if self.sweep_timer:
            self.sweep_timer.cancel()
            self.sweep_timer = None
        if self.mode != SWEEP:
            return
        self.get_logger().warn(f'[Mission] SWEEP timeout ({self.sweep_timeout_sec:.0f}s). Resuming Nav2 patrol.')
        self.state_pub.publish(String(data='STOP'))
        self.mode = PATROL
        if self.nav_mode == 'batch':
            self._send_batch()
        else:
            self._send_next()

    # ---------- Controller completion ----------
    def _on_simba_state(self, msg: String):
        if self.mode == SWEEP and msg.data.strip().upper() == 'STOP':
            if self.sweep_timer:
                self.sweep_timer.cancel()
                self.sweep_timer = None
            self.get_logger().info('[Mission] Search finished (STOP).')
            if self.resume_after:
                self.mode = PATROL
                if self.nav_mode == 'batch':
                    self._send_batch()
                else:
                    self._send_next()
            else:
                self.mode = PAUSED

    # ---------- Pose helper ----------
    def _mk_pose(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = Time()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = yaw_to_quat(math.radians(float(yaw_deg)))
        return ps

def main():
    rclpy.init()
    rclpy.spin(MissionManager())
    rclpy.shutdown()
