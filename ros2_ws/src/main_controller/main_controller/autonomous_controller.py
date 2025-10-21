import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class SearchMovementNode(Node):
    def __init__(self):
        super().__init__('autonomous_controller')

        # --- Publishers / Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/simba_state', 10)
        self.create_subscription(Float32, '/cps', self.cps_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/simba_state', self.state_callback, 10)

        # --- Parameters ---
        self.forward_speed = 0.2
        self.rotation_speed = 0.3  # rad/s
        self.sweep_range_deg = 20
        self.threshold_drive = 20.0
        self.threshold_goal = 100.0
        self.forward_duration = 0.5  # seconds

        # --- State machine ---
        self.state = "IDLE"
        self.current_cps = 0.0
        self.current_yaw = 0.0
        self.yaw_start = 0.0
        self.sweep_target_low = 0.0
        self.sweep_target_high = 0.0
        self.sweep_direction = 1
        self.sweep_data = []
        self.best_yaw = 0.0
        self.timer = self.create_timer(0.1, self.main_loop)
        self.motion_timer = None

        self.get_logger().info("SearchMovementNode started")

    # ---------------------- Callbacks ----------------------

    def cps_callback(self, msg: Float32):
        self.current_cps = msg.data

    def odom_callback(self, msg: Odometry):
        """Extract yaw (heading) from /odom quaternion."""
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def state_callback(self, msg: String):
        self.state = msg.data
        self.get_logger().info(f"State changed to: {self.state}")

    # ---------------------- Main loop ----------------------

    def main_loop(self):
        if self.state == "DRIVE":
            self.drive_state()
        elif self.state == "SWEEP":
            self.sweep_state()
        elif self.state == "REALIGN":
            self.realign_state()
        elif self.state == "FORWARD_SHORT":
            self.forward_short_state()
        elif self.state == "STOP":
            self.stop_state()

    # ---------------------- State handlers ----------------------

    def drive_state(self):
        """Drive forward until cps threshold is reached."""
        if self.current_cps < self.threshold_drive:
            twist = Twist()
            twist.linear.x = self.forward_speed
            self.cmd_pub.publish(twist)
        else:
            self.get_logger().info("Drive threshold reached → Sweep phase")
            self.stop_robot()
            # initialize sweep range
            self.yaw_start = self.current_yaw
            self.sweep_target_low = self.normalize_angle(self.yaw_start - math.radians(self.sweep_range_deg))
            self.sweep_target_high = self.normalize_angle(self.yaw_start + math.radians(self.sweep_range_deg))
            self.sweep_direction = 1  # start rotating positively
            self.sweep_data = []
            self.state_pub.publish(String(data="SWEEP"))

    def sweep_state(self):
        """Rotate across ±range around start yaw, recording cps values."""
        yaw = self.current_yaw
        twist = Twist()
        done = False

        # Determine sweep direction
        if self.sweep_direction == 1:  # sweeping positive direction
            twist.angular.z = abs(self.rotation_speed)
            if self.angle_reached(yaw, self.sweep_target_high, self.sweep_direction):
                done = True
        else:  # sweeping back negative
            twist.angular.z = -abs(self.rotation_speed)
            if self.angle_reached(yaw, self.sweep_target_low, self.sweep_direction):
                done = True

        self.cmd_pub.publish(twist)
        self.sweep_data.append((yaw, self.current_cps))

        if done:
            if self.sweep_direction == 1:
                # reverse direction for backward sweep
                self.sweep_direction = -1
                self.get_logger().info("Reached +range, sweeping back...")
            else:
                # full sweep done
                self.stop_robot()
                best_yaw, best_cps = max(self.sweep_data, key=lambda x: x[1])
                self.best_yaw = best_yaw
                self.get_logger().info(f"Sweep done. Best yaw={math.degrees(best_yaw):.1f}°, cps={best_cps:.2f}")
                self.state_pub.publish(String(data="REALIGN"))

    def realign_state(self):
        """Rotate robot to best yaw using odometry feedback."""
        yaw_error = self.angle_diff(self.best_yaw, self.current_yaw)
        if abs(yaw_error) > math.radians(2):
            twist = Twist()
            twist.angular.z = math.copysign(self.rotation_speed, yaw_error)
            self.cmd_pub.publish(twist)
        else:
            self.stop_robot()
            self.get_logger().info("Realigned to best yaw → moving forward shortly.")
            self.state_pub.publish(String(data="FORWARD_SHORT"))
            self.forward_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=self.forward_duration)

    def forward_short_state(self):
        """Move forward a bit toward chosen direction."""
        if self.get_clock().now() < self.forward_end_time:
            twist = Twist()
            twist.linear.x = self.forward_speed
            self.cmd_pub.publish(twist)
        else:
            self.stop_robot()
            if self.current_cps >= self.threshold_goal:
                self.get_logger().info("Goal threshold reached → STOP.")
                self.state_pub.publish(String(data="STOP"))
            else:
                self.get_logger().info("Restarting sweep phase.")
                self.state_pub.publish(String(data="SWEEP"))

    def stop_state(self):
        self.stop_robot()
        # stay idle

    # ---------------------- Helpers ----------------------

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def angle_diff(self, target, current):
        """Compute minimal signed difference target - current."""
        diff = target - current
        return math.atan2(math.sin(diff), math.cos(diff))

    def angle_reached(self, current, target, direction):
        """Check if we've passed target angle given rotation direction."""
        diff = self.angle_diff(target, current)
        # if direction positive: stop when diff < 0, if negative: stop when diff > 0
        return (direction == 1 and diff < 0) or (direction == -1 and diff > 0)


def main(args=None):
    rclpy.init(args=args)
    node = SearchMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()