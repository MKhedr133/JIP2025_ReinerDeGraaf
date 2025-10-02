import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DriveControlNode(Node):
    def __init__(self):
        super().__init__('drive_control_node')
        self.get_logger().info("Drive Control Node started")

        self.keyboardSubscriber = self.create_subscription(
            String,
            'key_input',
            self.key_input_callback,
            10
        )

        self.controlEnabledSubscriber = self.create_subscription(
            bool,
            'manual_control_enabled',
            self.control_enabled_callback,
            10
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriveControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
