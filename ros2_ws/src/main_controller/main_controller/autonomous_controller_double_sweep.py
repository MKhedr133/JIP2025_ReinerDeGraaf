import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SearchMovementNode(Node):
    def __init__(self):
        super().__init__('autonomous_controller')

        # --- QOS for pubs/subs ---
        # make sure QOS lines up with roomba stuff
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        # --- Publishers / Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/simba_state', 10)
        self.cps_sub = self.create_subscription(Float32, '/scintillator/cps', self.cps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.state_sub = self.create_subscription(String, '/simba_state', self.state_callback, 10)

        # --- Parameters ---
        self.forward_speed = 0.1
        self.rotation_speed = 0.1  # rad/s
        self.sweep_range_deg = 60
        self.sweep_increment_deg = 10.0
        self.sweep_wait_duration = 3.0  # seconds
        self.threshold_drive = 20.0
        self.threshold_goal = 500.0
        self.forward_duration = 0.5  # seconds

        # --- State machine ---
        self.state = "IDLE"
        self.target_yaw = 0.0
        self.current_cps = 0.0
        self.current_yaw = 0.0
        self.yaw_start = 0.0
        self.sweep_target_low = 0.0
        self.sweep_target_high = 0.0
        self.sweep_wait_end_time = None
        self.sweep_data = []
        self.best_yaw = 0.0
        self.sweep_pass = 1  # 1 for first pass (low->high), 2 for second pass (high->low)
        self.best_yaw_pass1 = None
        self.timer = self.create_timer(0.05, self.main_loop)
        self.motion_timer = None

        self.get_logger().info("SearchMovementNode started")

    # ---------------------- Callbacks ----------------------

    def cps_callback(self, msg: Float32):
        self.current_cps = msg.data
        self.get_logger().debug(f"Current CPS: {self.current_cps}")

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
        elif self.state == "SWEEP_START":
            self.sweep_start_state()
        elif self.state == "SWEEP_ALIGN":
            self.sweep_align_state()
        elif self.state == "SWEEP_INCREMENT":
            self.sweep_increment_state()
        elif self.state == "SWEEP_WAIT":
            self.sweep_wait_state()
        elif self.state == "SWEEP_EVALUATE":
            self.sweep_evaluate_state()
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
            self.state_pub.publish(String(data="SWEEP_START"))

    def sweep_start_state(self):
        """Initialize sweep parameters and start alignment."""
        if self.sweep_pass == 1:
            self.get_logger().info("Initializing sweep pass 1 (low to high).")
            self.yaw_start = self.current_yaw
            self.sweep_target_low = self.normalize_angle(self.yaw_start - math.radians(self.sweep_range_deg / 2.0))
            self.sweep_target_high = self.normalize_angle(self.yaw_start + math.radians(self.sweep_range_deg / 2.0))
            self.target_yaw = self.sweep_target_low
        else: # sweep_pass == 2
            self.get_logger().info("Initializing sweep pass 2 (high to low).")
            # Targets are already set from pass 1
            self.target_yaw = self.sweep_target_high

        self.sweep_data = []
        self.state_pub.publish(String(data="SWEEP_ALIGN"))

    def sweep_align_state(self):
        """Rotate to the starting angle for the current sweep pass."""
        self.get_logger().info(f"Aligning to sweep start angle: {math.degrees(self.target_yaw):.1f}°")
        if self.rotate_to_yaw(self.target_yaw):
            self.stop_robot()
            self.get_logger().info("Aligned. Starting incremental sweep.")
            self.state_pub.publish(String(data="SWEEP_WAIT"))
            self.sweep_wait_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=self.sweep_wait_duration)

    def sweep_increment_state(self):
        """Rotate by one increment."""
        self.get_logger().info(f"Sweeping to next increment: {math.degrees(self.target_yaw):.1f}°")
        if self.rotate_to_yaw(self.target_yaw):
            self.stop_robot()
            self.get_logger().info("Increment reached. Waiting to record data.")
            self.state_pub.publish(String(data="SWEEP_WAIT"))
            self.sweep_wait_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=self.sweep_wait_duration)

    def sweep_wait_state(self):
        """Record data whilst waiting a moment"""
        if self.get_clock().now() < self.sweep_wait_end_time:
            self.sweep_data.append((self.current_yaw, self.current_cps))
            self.get_logger().info(f"Recorded: yaw={math.degrees(self.current_yaw):.1f}°, cps={self.current_cps:.2f}")
            return

        # Check for sweep completion and prepare for next increment
        if self.sweep_pass == 1:
            # Sweeping from low to high
            if self.angle_diff(self.sweep_target_high, self.current_yaw) < math.radians(2.0):
                self.get_logger().info("Sweep pass 1 finished.")
                self.state_pub.publish(String(data="SWEEP_EVALUATE"))
            else:
                next_yaw = self.current_yaw + math.radians(self.sweep_increment_deg)
                self.target_yaw = self.normalize_angle(next_yaw)
                self.state_pub.publish(String(data="SWEEP_INCREMENT"))
        else: # sweep_pass == 2
            # Sweeping from high to low
            if self.angle_diff(self.sweep_target_low, self.current_yaw) < math.radians(2.0):
                self.get_logger().info("Sweep pass 2 finished.")
                self.state_pub.publish(String(data="SWEEP_EVALUATE"))
            else:
                next_yaw = self.current_yaw - math.radians(self.sweep_increment_deg)
                self.target_yaw = self.normalize_angle(next_yaw)
                self.state_pub.publish(String(data="SWEEP_INCREMENT"))

    def sweep_evaluate_state(self):
        """Find the best yaw from the sweep. For pass 2, average and transition to REALIGN."""
        self.stop_robot()
        if not self.sweep_data:
            self.get_logger().warning(f"No data from sweep pass {self.sweep_pass}, returning to DRIVE.")
            self.sweep_pass = 1 # Reset
            self.state_pub.publish(String(data="DRIVE"))
            return

        best_yaw_current_pass, best_cps = max(self.sweep_data, key=lambda x: x[1])

        if self.sweep_pass == 1:
            self.best_yaw_pass1 = best_yaw_current_pass
            self.get_logger().info(f"Sweep pass 1 done. Best yaw={math.degrees(self.best_yaw_pass1):.1f}°, cps={best_cps:.2f}")
            # Start pass 2
            self.sweep_pass = 2
            self.state_pub.publish(String(data="SWEEP_START"))
        else: # sweep_pass == 2
            best_yaw_pass2 = best_yaw_current_pass
            self.get_logger().info(f"Sweep pass 2 done. Best yaw={math.degrees(best_yaw_pass2):.1f}°, cps={best_cps:.2f}")

            # Average the best yaws from both passes
            self.best_yaw = self.normalize_angle((self.best_yaw_pass1 + best_yaw_pass2) / 2.0)
            self.get_logger().info(f"Final averaged best yaw: {math.degrees(self.best_yaw):.1f}°")
            self.sweep_pass = 1 # Reset for next time
            self.state_pub.publish(String(data="REALIGN"))

    def realign_state(self):
        """Rotate robot to best yaw using odometry feedback."""
        if self.rotate_to_yaw(self.best_yaw):
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
                self.state_pub.publish(String(data="SWEEP_START"))

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

    def rotate_to_yaw(self, target_yaw, tolerance_rad=math.radians(2.0)):
        """Rotates robot towards a target yaw. Returns True when aligned."""
        yaw_error = self.angle_diff(target_yaw, self.current_yaw)
        if abs(yaw_error) > tolerance_rad:
            twist = Twist()
            # Use a slightly faster rotation for alignment
            twist.angular.z = math.copysign(self.rotation_speed * 5, yaw_error)
            self.cmd_pub.publish(twist)
            return False
        else:
            self.stop_robot()
            return True


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