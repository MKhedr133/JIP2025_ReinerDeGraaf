import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Initial state
        self.state = 'IDLE'
        self.get_logger().info(f"Initial state: {self.state}")

        # Subscriber to /key_input
        self.subscription = self.create_subscription(
            String,
            '/key_input',
            self.key_input_callback,
            10
        )

    def key_input_callback(self, msg: String):
        key = msg.data.strip()
        self.get_logger().info(f"Received key input: '{key}'")

        if key == 't':
            self.state = 'TRIGGERED'
            self.get_logger().info(f"State changed to: {self.state}")
        else:
            self.get_logger().info(f"No state change (current state: {self.state})")


def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
