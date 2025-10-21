import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import AngularServo

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')

        # Declare and get parameter for GPIO pin (default GPIO18)
        self.declare_parameter('gpio_pin', 18)
        self.gpio_pin = self.get_parameter('gpio_pin').value

        # Initialize AngularServo with safe defaults for QY3225MG
        try:
            self.servo = AngularServo(
                self.gpio_pin,
                min_angle=0,
                max_angle=180,
                min_pulse_width=0.0005,
                max_pulse_width=0.0025
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize AngularServo on GPIO {self.gpio_pin}: {e}")
            exit(1)

        self.subscriber = self.create_subscription(
            Float64,
            'servo_angle',
            self.angle_callback,
            10
        )
        self.get_logger().info(f"AngularServo controller initialized on GPIO {self.gpio_pin}.")

    def angle_callback(self, msg):
        angle = msg.data
        clamped_angle = max(0.0, min(180.0, angle))  # Keep within safe range

        try:
            self.servo.angle = clamped_angle
            self.get_logger().info(f"Set angle: {clamped_angle:.1f}°")
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
