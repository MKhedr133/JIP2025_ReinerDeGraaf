#!/usr/bin/env python3
"""
Simple ROS 2 servo controller using gpiozero on GPIO18.

Topics:
  - /servo/angle_deg  (std_msgs/Float32): desired angle in degrees
  - /servo/enable     (std_msgs/Bool): True=enable PWM, False=disable

Main params (override in launch):
  - gpio_pin (int)          : BCM pin number (default 18)
  - min_pulse_us (int)      : microseconds at min angle (default 1500)
  - max_pulse_us (int)      : microseconds at max angle (default 2500)
  - min_angle_deg (float)   : minimum angle (default 0)
  - max_angle_deg (float)   : maximum angle (default 90)
  - startup_angle_deg (float): initial angle on start (default 45)
"""

from std_msgs.msg import Float32, Bool
import rclpy
from rclpy.node import Node

try:
    from gpiozero import AngularServo
except Exception as e:
    AngularServo = None
    _GPIOZERO_ERR = e


class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_controller")

        # --- Parameters (simple & visible) ---
        self.declare_parameter("gpio_pin", 18)
        self.declare_parameter("min_pulse_us", 1500)
        self.declare_parameter("max_pulse_us", 2500)
        self.declare_parameter("min_angle_deg", 0.0)
        self.declare_parameter("max_angle_deg", 90.0)
        self.declare_parameter("startup_angle_deg", 45.0)

        pin = int(self.get_parameter("gpio_pin").value)
        min_us = int(self.get_parameter("min_pulse_us").value)
        max_us = int(self.get_parameter("max_pulse_us").value)
        min_deg = float(self.get_parameter("min_angle_deg").value)
        max_deg = float(self.get_parameter("max_angle_deg").value)
        startup_deg = float(self.get_parameter("startup_angle_deg").value)

        if AngularServo is None:
            self.get_logger().error(f"gpiozero not available: {_GPIOZERO_ERR}")
            raise RuntimeError("Install gpiozero on the Pi (and run with GPIO access).")

        # Convert μs → seconds (what gpiozero expects)
        min_pw_s = min_us / 1_000_000.0
        max_pw_s = max_us / 1_000_000.0

        # --- Create the servo (super direct) ---
        self.servo = AngularServo(
            pin,
            min_angle=min_deg,
            max_angle=max_deg,
            min_pulse_width=min_pw_s,
            max_pulse_width=max_pw_s,
            frame_width=0.020,  # 20 ms typical
        )

        # Set initial angle and mark enabled
        self.enabled = True
        self._set_angle(self._clamp(startup_deg, min_deg, max_deg))

        # --- Subscriptions (tiny + obvious) ---
        self.create_subscription(Float32, "servo/angle_deg", self.on_angle, 10)
        self.create_subscription(Bool, "servo/enable", self.on_enable, 10)

        self.get_logger().info(
            f"Servo ready on GPIO{pin}: angle {min_deg}..{max_deg}° "
            f"mapped to {min_us}..{max_us} μs (enabled)"
        )

    # ---------- Callbacks ----------
    def on_angle(self, msg: Float32):
        if not self.enabled:
            self.get_logger().warn("Ignored angle: servo is disabled. Publish /servo/enable true.")
            return
        self._set_angle(float(msg.data))

    def on_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.servo.attach()   # start PWM
            self.enabled = True
            self.get_logger().info("Servo enabled (attached).")
        elif (not msg.data) and self.enabled:
            self.servo.detach()   # stop PWM
            self.enabled = False
            self.get_logger().info("Servo disabled (detached).")

    # ---------- Tiny helpers ----------
    def _set_angle(self, angle_deg: float):
        try:
            self.servo.angle = angle_deg
        except Exception as e:
            self.get_logger().error(f"Failed to set angle: {e}")

    @staticmethod
    def _clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    # Clean shutdown = release the servo
    def destroy_node(self):
        try:
            self.servo.detach()
            self.servo.close()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = ServoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
