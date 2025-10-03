"""
Main Controller Node

This ROS 2 node serves as the main control center of the application. It listens to incoming data from other nodes 
and makes decisions based on the received information. Specifically, it subscribes to the `/key_input` topic, 
which provides keyboard inputs, and updates its internal state accordingly.

### Functionality:
- Subscribes to the `/key_input` topic to receive keyboard inputs.
- Maintains an internal state (`IDLE` by default).
- Changes the state to `TRIGGERED` when the `t` key is received.
- Logs all received inputs and state changes for debugging and monitoring.

### Subscribed Topics:
- `/key_input` (std_msgs/String): Receives keyboard inputs as string messages.

### States:
- `IDLE`: The initial state of the node.
- `TRIGGERED`: The state changes to this when the `t` key is received.

"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import WheelTicks
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math


class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Initial state
        self.state = 'IDLE'
        self.get_logger().info(f"Initial state: {self.state}")

        # Keep track of Distance Travelled in EXPERIMENT state
        self.startpose = (0.0,0.0,0.0) # TODO: save this in a better data format
        self.startTickLeft = 0
        self.startTickRight = 0
        
        # Subscriber to /key_input
        self.keyboardSubscriber = self.create_subscription(
            String,
            'key_input',
            self.key_input_callback,
            10
        )

        # make sure QOS lines up with roomba stuff
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscriber to /odom
        self.odomSubscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        
        # Subscriber to /wheel_ticks
        self.tickSubscriber = self.create_subscription(
            WheelTicks,
            'wheel_ticks',
            self.tick_callback,
            qos)
        
        # Publish whether MANUAL control is enabled on /manual_control_enabled
        self.manualControlEnabled = True
        self.manualControlPublisher = self.create_publisher(
            Bool,
            'manual_control_enabled',
            10
        )

    def key_input_callback(self, msg: String):
        key = msg.data.strip()
        self.get_logger().info(f"Received key input: '{key}'")

        if key == 'y':
            self.state = 'EXPERIMENT'
            self.get_logger().info(f"State changed to: {self.state}. Starting experiment.")
        elif key == 'h':
            self.state = 'IDLE'
            self.get_logger().info(f"State changed to: {self.state}. Stopping experiment.")
        elif key == 'm':
            # Toggle manual control
            self.manualControlEnabled = not (self.manualControlEnabled)
            msg_flag = Bool()
            msg_flag.data = self.manualControlEnabled
            self.manualControlPublisher.publish(msg_flag)
            self.get_logger().info(f"Manual control {'enabled' if self.manualControlEnabled else 'disabled'}. Current state: {self.state}")
        else:
            self.get_logger().info(f"No state change (current state: {self.state})")

    # PROBABLY DEPRECATED, Wheelticks more accurate!
    def odom_callback(self, msg: Odometry):
        if self.state == 'EXPERIMENT':
            # Extract position from the Odometry message
            position = msg.pose.pose.position
            # Log how much distance we've travelled
            distance_travelled = math.sqrt((msg.pose.pose.position.x - self.startpose[0]) ** 2 +
                                  (msg.pose.pose.position.y - self.startpose[1]) ** 2) ** 0.5
            # self.get_logger().info(f"ODOM distance: {distance_travelled:.2f} meters")
        elif self.state == 'IDLE':
            # Update startpose to current position
            self.startpose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        
    def tick_callback(self, msg: WheelTicks):
        if self.state == 'EXPERIMENT':
            ticks_travelled_avg = ((msg.ticks_left - self.startTickLeft) + (msg.ticks_right - self.startTickRight)) / 2.0
            circumference = math.pi * 72.0  # in mm
            dist_travelled = ticks_travelled_avg / 508.8 * circumference
            # Log wheel ticks
            self.get_logger().info(f"WHEELTICK distance {dist_travelled / 100.0} cm")
        elif self.state == 'IDLE':
            # Update start ticks to current ticks
            self.startTickLeft = msg.ticks_left
            self.startTickRight = msg.ticks_right

def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
