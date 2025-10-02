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
