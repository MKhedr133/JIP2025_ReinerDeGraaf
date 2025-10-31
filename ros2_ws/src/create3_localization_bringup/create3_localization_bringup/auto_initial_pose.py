#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_srvs.srv import Empty
from tf_transformations import quaternion_from_euler

LM_SVC = '/lifecycle_manager_localization/get_state'

def q_from_yaw(yaw):
    x, y, z, w = quaternion_from_euler(0.0, 0.0, yaw)
    q = Quaternion(); q.x, q.y, q.z, q.w = x, y, z, w
    return q

class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')

        self.declare_parameter('mode', 'auto')        # 'auto' => global_localization; 'pose' => publish pose
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw_deg', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('wait_timeout_sec', 30.0)

        self._lm_cli = self.create_client(GetState, LM_SVC)
        self._glob_cli = self.create_client(Empty, '/global_localization')

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST
        self._init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

        self.get_logger().info('auto_initial_pose started')
        self.create_timer(0.5, self._tick)

        self._done = False
        self._started = time.time()

    def _tick(self):
        if self._done:
            return

        # 1) Wait for lifecycle manager to be reachable
        if not self._lm_cli.wait_for_service(timeout_sec=0.1):
            return

        # Query state safely (ROS 2 rclpy requires call_async)
        req = GetState.Request()
        future = self._lm_cli.call_async(req)
        if not rclpy.spin_until_future_complete(self, future, timeout_sec=0.5):
            return
        resp = future.result()
        if resp is None or resp.current_state.label != 'active':
            # Not active yet
            if time.time() - self._started > self.get_parameter('wait_timeout_sec').value:
                self.get_logger().warn('Timed out waiting for lifecycle to activate; proceeding anyway.')
                self._proceed()
            return

        # Active -> proceed
        self._proceed()

    def _proceed(self):
        if self._done:
            return
        mode = self.get_parameter('mode').get_parameter_value().string_value
        if mode == 'pose':
            self._publish_pose()
        else:
            self._call_global_localization()
        self._done = True

    def _call_global_localization(self):
        if not self._glob_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('AMCL /global_localization service not available.')
            return
        fut = self._glob_cli.call_async(Empty.Request())
        if rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0):
            self.get_logger().info('Called /global_localization on AMCL.')
        else:
            self.get_logger().error('Timeout calling /global_localization.')

    def _publish_pose(self):
        x = float(self.get_parameter('initial_x').value)
        y = float(self.get_parameter('initial_y').value)
        yaw = math.radians(float(self.get_parameter('initial_yaw_deg').value))
        frame_id = self.get_parameter('frame_id').value

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = q_from_yaw(yaw)

        # Modest covariance (tune as needed)
        cov = [0.0]*36
        cov[0] = 0.25   # x var (0.5 m)^2
        cov[7] = 0.25   # y var
        cov[35] = (math.radians(15.0))**2  # yaw var
        msg.pose.covariance = cov

        self._init_pub.publish(msg)
        self.get_logger().info(f'Published /initialpose at ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')

def main():
    rclpy.init()
    node = AutoInitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
