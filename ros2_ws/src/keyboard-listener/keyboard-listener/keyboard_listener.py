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

            self.get_logger().info(f"Key pressed: {key_str}")
            msg = String()
            msg.data = key_str
            self.publisher_.publish(msg)    #publish the key press

            if key == keyboard.Key.esc:
                self.get_logger().info("ESC pressed. Shutting down.")
                rclpy.shutdown()

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
