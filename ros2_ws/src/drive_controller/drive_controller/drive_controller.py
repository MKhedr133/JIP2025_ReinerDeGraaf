"""
Manual Drive Control Node

This ROS 2 node provides manual teleoperation control for the robot using
keyboard inputs. It translates specific key presses into velocity commands
to drive and rotate the robot.

The node's functionality can be enabled or disabled via an external topic,
allowing other nodes to take control of the robot's movement (e.g., for
autonomous behavior) and prevent conflicting commands.

### Functionality:
- **Movement**:
  - 'w': Move forward
  - 's': Move backward
  - 'a': Rotate counter-clockwise (left)
  - 'd': Rotate clockwise (right)
- **Speed Adjustment**:
  - 'r'/'f': Increase/decrease linear speed.
  - 't'/'g': Increase/decrease angular speed.

### Subscribed Topics:
- `/key_input` (std_msgs/String): Receives keyboard presses.
- `/manual_control_enabled` (std_msgs/Bool): A flag to enable or disable the node's operation.

### Published Topics:
- `/cmd_vel` (geometry_msgs/Twist): Publishes linear and angular velocity commands.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class DriveControlNode(Node):
    def __init__(self):
        super().__init__('drive_control')
        self.get_logger().info("Drive Control Node started")

        self.keyboardSubscriber = self.create_subscription(
            String,
            'key_input',
            self.key_input_callback,
            10
        )

        self.controlEnabledSubscriber = self.create_subscription(
            Bool,
            'manual_control_enabled',
            self.control_enabled_callback,
            10
        )

        self.movePublisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.controlEnabled = True # Whether manual control is enabled
        self.lin_speed = 0.1  # Linear speed
        self.ang_speed = 0.5  # Angular speed

    def move_forward(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.movePublisher.publish(msg)

    def rotate(self, speed):
        msg = Twist()
        msg.angular.z = speed
        self.movePublisher.publish(msg)

    def control_enabled_callback(self, msg: Bool):
        self.controlEnabled = msg.data
        if msg.data:
            self.get_logger().info("Manual control enabled")
        else:
            self.get_logger().info("Manual control disabled")

    def key_input_callback(self, msg: String):
        if not self.controlEnabled:
            return
        
        try:
            if msg.data == 'w':
                self.move_forward(self.lin_speed)
            elif msg.data == 'a':
                self.rotate(self.ang_speed)
            elif msg.data == 's':
                self.move_forward(-self.lin_speed)
            elif msg.data == 'd':
                self.rotate(-self.ang_speed)
            elif msg.data == 'r':
                self.lin_speed += 0.01
                self.get_logger().info(f"Linear speed increased to {self.lin_speed:.2f}")
            elif msg.data == 'f':
                self.lin_speed = max(0.01, self.lin_speed - 0.01)
                self.get_logger().info(f"Linear speed decreased to {self.lin_speed:.2f}")
            elif msg.data == 't':
                self.ang_speed += 0.01
            elif msg.data == 'g':
                self.ang_speed = max(0.01, self.ang_speed - 0.01)
        except AttributeError:
            print('Special key {0} pressed'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = DriveControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
