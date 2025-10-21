import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import Servo

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')

        # Declare and get parameter for GPIO pin (default GPIO18)
        self.declare_parameter('gpio_pin', 18)
        self.gpio_pin = self.get_parameter('gpio_pin').value

        # Initialize gpiozero Servo
        try:
            self.servo = Servo(self.gpio_pin, min_pulse_width=0.0005, max_pulse_width=0.0025) # TODO: Adjust pulse widths as needed
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Servo on GPIO {self.gpio_pin}: {e}")
            exit(1)

        self.subscriber = self.create_subscription(
            Float64,
            'servo_angle',
            self.angle_callback,
            10
        )
        self.get_logger().info(f"Servo controller initialized on GPIO {self.gpio_pin} using gpiozero.")

    def angle_callback(self, msg):
        angle = msg.data
        clamped_angle = max(0.0, min(180.0, angle))
        servo_value = (clamped_angle - 90.0) / 90.0  # Map 0-180 → -1 to 1

        try:
            self.servo.value = servo_value
            self.get_logger().info(f"Set angle: {clamped_angle:.1f}°, gpiozero value: {servo_value:.2f}")
        except Exception as e:
            self.get_logger().error(f"Failed to set servo angle: {e}")

    def destroy_node(self):
        self.servo.detach()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
