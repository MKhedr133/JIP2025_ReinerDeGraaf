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
        param_defaults = {
            'forward_speed': 0.1,
            'rotation_speed': 0.1,
            'sweep_rotation_speed': 0.1, # 0.005 is slowest it goes
            'sweep_range_deg': 60.0,
            'sweep_increment_deg': 10.0,
            'sweep_wait_duration': 3.0,
            'threshold_drive': 20.0,
            'threshold_goal': 500.0,
            'forward_duration': 0.5,
            'min_forward_duration': 5.0,
            'max_forward_duration': 5.0,
        }

        for param_name, default_value in param_defaults.items():
            self.declare_parameter(param_name, default_value)
            setattr(self, param_name, self.get_parameter(param_name).value)

        # --- State machine ---
        self.state = "IDLE"
        self.target_yaw = 0.0
        self.current_cps = 0.0
        self.current_yaw = 0.0
        self.yaw_start = 0.0
        self.sweep_target_low = 0.0
        self.sweep_target_high = 0.0
        self.best_yaw = 0.0
        self.best_cps = 0.0
        self.forward_end_time = None
        self.last_peak_cps = None
        
        # Continuous sweep variables
        self.best_yaw_pass1 = None
        self.best_cps_pass1 = -1.0
        self.best_yaw_pass2 = None
        self.best_cps_pass2 = -1.0

        self.timer = self.create_timer(0.05, self.main_loop)

        self.get_logger().info("Double Sweep started")

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
        elif self.state == "SWEEPING_PASS_1":
            self.sweeping_pass_1_state()
        elif self.state == "SWEEPING_PASS_2":
            self.sweeping_pass_2_state()
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
        self.get_logger().info("Initializing sweep.")
        self.yaw_start = self.current_yaw
        self.sweep_target_low = self.normalize_angle(self.yaw_start - math.radians(self.sweep_range_deg / 2.0))
        self.sweep_target_high = self.normalize_angle(self.yaw_start + math.radians(self.sweep_range_deg / 2.0))
        
        # Reset sweep data
        self.best_cps_pass1 = -1.0
        self.best_yaw_pass1 = None
        self.best_cps_pass2 = -1.0
        self.best_yaw_pass2 = None

        # Transition to alignment state
        self.get_logger().info(f"Aligning to sweep start: {math.degrees(self.sweep_target_low):.1f}°")
        self.state_pub.publish(String(data="SWEEP_ALIGN"))

    def sweep_align_state(self):
        """Rotate to the starting angle for the sweep."""
        if self.rotate_to_yaw(self.sweep_target_low):
            self.stop_robot()
            self.get_logger().info("Aligned. Starting sweep pass 1.")
            self.state_pub.publish(String(data="SWEEPING_PASS_1"))

    def sweeping_pass_1_state(self):
        """Continuously rotate from low to high, recording the best CPS."""
        # Check if we have reached the target
        if abs(self.angle_diff(self.sweep_target_high, self.current_yaw)) < math.radians(0.5):
            self.stop_robot()
            self.get_logger().info("Sweep pass 1 finished. Starting pass 2.")
            self.state_pub.publish(String(data="SWEEPING_PASS_2"))
            return

        # Record best CPS reading during the sweep
        if self.current_cps > self.best_cps_pass1:
            self.best_cps_pass1 = self.current_cps
            self.best_yaw_pass1 = self.current_yaw

        # Rotate slowly
        twist = Twist()
        twist.angular.z = self.sweep_rotation_speed
        self.cmd_pub.publish(twist)
        self.get_logger().debug(f"Sweeping pass 1: yaw={math.degrees(self.current_yaw):.1f}°, cps={self.current_cps:.2f}")

    def sweeping_pass_2_state(self):
        """Continuously rotate from high to low, recording the best CPS."""
        # Check if we have reached the target
        if abs(self.angle_diff(self.sweep_target_low, self.current_yaw)) < math.radians(0.5):
            self.stop_robot()
            self.get_logger().info("Sweep pass 2 finished. Evaluating.")
            self.state_pub.publish(String(data="SWEEP_EVALUATE"))
            return

        # Record best CPS reading during the sweep
        if self.current_cps > self.best_cps_pass2:
            self.best_cps_pass2 = self.current_cps
            self.best_yaw_pass2 = self.current_yaw

        # Rotate slowly in the opposite direction
        twist = Twist()
        twist.angular.z = -self.sweep_rotation_speed
        self.cmd_pub.publish(twist)

    def sweep_evaluate_state(self):
        """Find the best yaw from the sweep. For pass 2, average and transition to REALIGN."""
        self.stop_robot()
        if self.best_yaw_pass1 is None or self.best_yaw_pass2 is None:
            self.get_logger().warning("Sweep did not yield valid data, returning to DRIVE.")
            self.state_pub.publish(String(data="DRIVE"))
            return

        self.get_logger().info(f"Pass 1 best: yaw={math.degrees(self.best_yaw_pass1):.1f}°, cps={self.best_cps_pass1:.2f}")
        self.get_logger().info(f"Pass 2 best: yaw={math.degrees(self.best_yaw_pass2):.1f}°, cps={self.best_cps_pass2:.2f}")

        # Average the best yaws from both passes
        # Handle wrap-around for averaging angles
        avg_x = math.cos(self.best_yaw_pass1) + math.cos(self.best_yaw_pass2)
        avg_y = math.sin(self.best_yaw_pass1) + math.sin(self.best_yaw_pass2)
        self.best_yaw = math.atan2(avg_y, avg_x)

        # Use the average for two values of the two peak CPS values for the forward duration calculation
        self.best_cps = (self.best_cps_pass1 + self.best_cps_pass2) / 2.0

        self.get_logger().info(f"Final averaged best yaw: {math.degrees(self.best_yaw):.1f}°")
        self.state_pub.publish(String(data="REALIGN"))

    def realign_state(self):
        """Rotate robot to best yaw using odometry feedback."""
        if self.rotate_to_yaw(self.best_yaw):
            self.stop_robot()
            self.get_logger().info("Realigned to best yaw → moving forward shortly.")
            self.state_pub.publish(String(data="FORWARD_SHORT"))
            
            self.forward_end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=self.forward_duration)

            if self.last_peak_cps is not None:
                cps_ratio = float(self.last_peak_cps) / float(self.current_cps)
                distance_ratio = (cps_ratio)**0.5  # Inverse square law
                adjusted_duration = self.forward_duration * distance_ratio
                
                # Limit the adjusted duration to avoid extreme values
                adjusted_duration = max(0.1, min(adjusted_duration, self.forward_duration * 2.0))

                self.forward_end_time = self.get_clock().now() + rclpy.time.Duration(seconds=adjusted_duration)
                self.get_logger().info(f"Adjusting forward duration to: {adjusted_duration:.2f} seconds.")
            else:
                self.get_logger().warn("No previous peak CPS recorded.  Using default forward duration.")

    def forward_short_state(self):
        """Move forward a bit toward chosen direction, adjusting duration based on inverse square law."""
        if self.get_clock().now() < self.forward_end_time:
            twist = Twist()
            twist.linear.x = self.forward_speed
            self.cmd_pub.publish(twist)
            if self.current_cps >= self.threshold_goal:
                self.stop_robot()
                self.get_logger().info("Goal threshold reached → STOP.")
                self.state_pub.publish(String(data="STOP"))
        else:
            self.stop_robot()
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
            twist.angular.z = math.copysign(self.rotation_speed, yaw_error)
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