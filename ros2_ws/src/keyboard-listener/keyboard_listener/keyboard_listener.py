"""
Keyboard Listener Node

This ROS 2 node listens for keyboard inputs and publishes the pressed keys to the `/key_input` topic.
It uses the `pynput` library to capture keyboard events and publishes them as `std_msgs/String` messages.

### Functionality:
- Captures alphanumeric keys and special keys (e.g., space, escape).
- Publishes each key press to the `/key_input` topic.
- Shuts down the node when the `ESC` key is pressed.

### Published Topics:
- `/key_input` (std_msgs/String): Publishes the string representation of the pressed key.

"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
import threading


class KeyboardListenerNode(Node):
    def __init__(self):
        super().__init__("KeyboardListener")
        self.publisher_ = self.create_publisher(String, 'key_input', 10)    #publisher for key inputs
        self.get_logger().info("Keyboard listener node started. Press ESC to quit.")

        # Start keyboard listener in background thread
        self.listener_thread = threading.Thread(target=self.start_keyboard_listener, daemon=True)
        self.listener_thread.start()

    def start_keyboard_listener(self):
        def on_press(key):
            try:
                key_str = key.char  # Alphanumeric keys
            except AttributeError:
                key_str = str(key)  # Special keys (Key.esc, Key.space, etc.)

            # self.get_logger().info(f"Key pressed: {key_str}")
            msg = String()
            msg.data = key_str
            self.publisher_.publish(msg)    #publish the key press

            if key == keyboard.Key.esc:
                # self.get_logger().info("ESC pressed. Shutting down.")
                rclpy.shutdown()

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
