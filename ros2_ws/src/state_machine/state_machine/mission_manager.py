#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from typing import List, Dict

from .utils import yaw_to_quat
from .route_generators import boustrophedon_sweep

PATROL = 'PATROL'
SWEEP  = 'SWEEP'
PAUSED = 'PAUSED'

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # ---------------- Params ----------------
        self.declare_parameter('cps_topic', '/scintillator/cps')
        self.declare_parameter('cps_trigger', 20.0)
        self.declare_parameter('frame_id', 'map')

        # Waypoint source: 'generated' | 'yaml' | 'list'
        self.declare_parameter('route_source', 'generated')
        self.declare_parameter('map_yaml', '/work/config/maps/create3_home_map.yaml')
        self.declare_parameter('lane_spacing_m', 1.0)
        self.declare_parameter('heading_deg', 0.0)

        # If route_source == 'list' (inline) or 'yaml' (load file of [{'x','y','theta_deg'}...])
        self.declare_parameter('waypoints', [])

        # Nav mode: 'sequential' (NavigateToPose) or 'batch' (NavigateThroughPoses)
        self.declare_parameter('nav_mode', 'sequential')

        # Resume after SWEEP STOP?
        self.declare_parameter('resume_patrol_after_sweep', True)

        self.cps_topic = self.get_parameter('cps_topic').value
        self.cps_trigger = float(self.get_parameter('cps_trigger').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.route_source = str(self.get_parameter('route_source').value)
        self.map_yaml = str(self.get_parameter('map_yaml').value)
        self.lane_spacing_m = float(self.get_parameter('lane_spacing_m').value)
        self.heading_deg = float(self.get_parameter('heading_deg').value)
        self.nav_mode = str(self.get_parameter('nav_mode').value)
        self.resume_after = bool(self.get_parameter('resume_patrol_after_sweep').value)
        self.inline_wps = self.get_parameter('waypoints').value

        # ---------------- IO ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.cps_sub = self.create_subscription(Float32, self.cps_topic, self.cps_cb, qos)
        self.state_pub = self.create_publisher(String, '/simba_state', 10)
        self.state_sub = self.create_subscription(String, '/simba_state', self.simba_state_cb, 10)

        # ---------------- Actions ----------------
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nav_through_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.current_goal_handle = None

        # ---------------- State ----------------
        self.mode = PATROL
        self.patrol_index = 0
        self.patrol: List[Dict] = self.build_patrol()

        # Kick off once servers are up
        self.timer = self.create_timer(0.5, self.start_once)
        self.get_logger().info(f'[Mission] Ready. Using {len(self.patrol)} patrol waypoints. Mode={self.nav_mode}')

    # ---------------- Patrol building ----------------
    def build_patrol(self) -> List[Dict]:
        if self.route_source == 'generated':
            wps = boustrophedon_sweep(
                map_yaml_path=self.map_yaml,
                lane_spacing_m=self.lane_spacing_m,
                heading_deg=self.heading_deg
            )
            if not wps:
                self.get_logger().warn('Generated zero waypoints; fallback to inline list (if any).')
                wps = self.inline_wps or []
            return wps
        elif self.route_source == 'yaml':
            import yaml, os
            with open(self.map_yaml, 'r') as f:
                data = yaml.safe_load(f)
            # If this YAML is a map, the generator above should be used.
            # If this YAML is a list of waypoints, accept it.
            if isinstance(data, list):
                return data
            else:
                self.get_logger().warn('YAML not a waypoint list; generating from map instead.')
                return boustrophedon_sweep(self.map_yaml, self.lane_spacing_m, heading_deg=self.heading_deg)
        elif self.route_source == 'list':
            return self.inline_wps or []
        else:
            self.get_logger().warn('Unknown route_source; returning empty patrol.')
            return []

    # ---------------- Lifecycle ----------------
    def start_once(self):
        self.timer.cancel()
        # Ensure correct action server is used
        if self.nav_mode == 'batch':
            self.get_logger().info('Waiting for /navigate_through_poses...')
            self.nav_through_client.wait_for_server()
            self.send_batch()
        else:
            self.get_logger().info('Waiting for /navigate_to_pose...')
            self.nav_to_pose_client.wait_for_server()
            self.send_next()

    # ---------------- Helpers ----------------
    def make_pose(self, x, y, yaw_deg) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = Time()  # leave 0; Nav2 only cares about frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation = yaw_to_quat(math.radians(float(yaw_deg)))
        return ps

    # ---------------- Nav: sequential (NavigateToPose) ----------------
    def send_next(self):
        if self.mode != PATROL:
            return
        if self.patrol_index >= len(self.patrol):
            self.get_logger().info('[Mission] Patrol complete. Looping from start.')
            self.patrol_index = 0
        if not self.patrol:
            self.get_logger().error('[Mission] No patrol waypoints configured.')
            return
        wp = self.patrol[self.patrol_index]
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(wp['x'], wp['y'], wp.get('theta_deg', 0.0))
        self.get_logger().info(f'[Mission] → Goal {self.patrol_index+1}/{len(self.patrol)}: '
                               f"({wp['x']:.2f}, {wp['y']:.2f}) θ={wp.get('theta_deg',0):.1f}°")
        future = self.nav_to_pose_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('[Mission] Goal rejected by Nav2. Trying next...')
            self.patrol_index += 1
            self.send_next()
            return
        self.current_goal_handle = handle
        self.get_logger().info('[Mission] Goal accepted.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        if self.mode != PATROL:
            return
        status = future.result().status
        self.get_logger().info(f'[Mission] Goal finished with status={status}.')
        self.patrol_index += 1
        self.send_next()

    # ---------------- Nav: batch (NavigateThroughPoses) ----------------
    def send_batch(self):
        if self.mode != PATROL:
            return
        if not self.patrol:
            self.get_logger().error('[Mission] No patrol waypoints configured.')
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.make_pose(w['x'], w['y'], w.get('theta_deg', 0.0)) for w in self.patrol]
        self.get_logger().info(f'[Mission] → Batch of {len(goal.poses)} poses.')
        future = self.nav_through_client.send_goal_async(goal)
        future.add_done_callback(self.batch_goal_response_cb)

    def batch_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('[Mission] Batch goal rejected.')
            return
        self.current_goal_handle = handle
        self.get_logger().info('[Mission] Batch goal accepted.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.batch_result_cb)

    def batch_result_cb(self, future):
        if self.mode != PATROL:
            return
        self.get_logger().info('[Mission] Batch finished.')
        # Loop forever
        self.send_batch()

    # ---------------- CPS trigger & handoff ----------------
    def cps_cb(self, msg: Float32):
        if self.mode != PATROL:
            return
        if msg.data >= self.cps_trigger:
            self.get_logger().warn(f'[Mission] CPS {msg.data:.1f} ≥ {self.cps_trigger} → HANDOFF to autonomous controller.')
            self.mode = SWEEP
            # Cancel Nav2 goal (either client)
            if self.current_goal_handle:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _: self.get_logger().info('[Mission] Nav2 goal canceled.'))
            # Tell your autonomous controller to begin (DRIVE → it will enter SWEEP states)
            self.state_pub.publish(String(data='DRIVE'))

    # ---------------- Listen to autonomous controller completion ----------------
    def simba_state_cb(self, msg: String):
        # When autonomous_controller finishes, it publishes 'STOP' (per your logic)
        if self.mode == SWEEP and msg.data.strip().upper() == 'STOP':
            self.get_logger().info('[Mission] SWEEP finished (STOP).')
            if self.resume_after:
                self.get_logger().info('[Mission] Resuming patrol.')
                self.mode = PATROL
                # Reset goal handle and continue
                self.current_goal_handle = None
                if self.nav_mode == 'batch':
                    self.send_batch()
                else:
                    self.send_next()
            else:
                self.get_logger().info('[Mission] Holding after sweep.')
                self.mode = PAUSED

def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
