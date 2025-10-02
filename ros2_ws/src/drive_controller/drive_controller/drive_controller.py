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
        self.controlEnabled = True

        self.movePublisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

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

    def control_enabled_callback(self, msg: bool):
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
                lin_speed += 0.01
            elif msg.data == 'f':
                lin_speed = max(0.01, self.lin_speed - 0.01)
            elif msg.data == 't':
                ang_speed += 0.01
            elif msg.data == 'g':
                ang_speed = max(0.01, self.ang_speed - 0.01)
        except AttributeError:
            print('Special key {0} pressed'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = DriveControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
