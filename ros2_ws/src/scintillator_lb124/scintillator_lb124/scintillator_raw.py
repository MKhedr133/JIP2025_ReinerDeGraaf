#!/usr/bin/env python3

"""
Raw serial -> /scintillator/raw
-----------------------------------
Reads text lines from the LB‑124 over a serial port and publishes them as `std_msgs/String`
on `/scintillator/raw`.

Make sure the device is set to
continuous/periodic output (or press its "store" button if needed)

Parameters
- port (string, default "/dev/ttyUSB0")
- baud (int, default 19200)
- timeout_s (double, default 1.0) — serial timeout

Topics
- /scintillator/raw (std_msgs/String) — raw lines from serial
"""


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class ScintillatorRawNode(Node):
    def __init__(self):
        super().__init__("scintillator_raw")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 19200)
        self.declare_parameter("timeout_s", 1.0)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = int(self.get_parameter("baud").get_parameter_value().integer_value)
        timeout_s = float(self.get_parameter("timeout_s").get_parameter_value().double_value)

        try:
            self.ser = serial.Serial(port, baud, timeout=timeout_s)
        except Exception as e:
            self.get_logger().fatal(f"Cannot open serial {port}@{baud}: {e}")
            raise

        self.pub = self.create_publisher(String, "/scintillator/raw", 10)
        # Read roughly every 10 ms (non-blocking thanks to serial timeout)
        self.create_timer(0.01, self._tick)
        self.get_logger().info(f"Serial open: {port}@{baud}, publishing to /scintillator/raw")

    def _tick(self):
        try:
            raw = self.ser.readline()  # bytes (empty if timeout)
        except Exception as e:
            self.get_logger().warn(f"Serial read failed: {e}")
            return
        if not raw:
            return
        msg = String()
        msg.data = raw.decode("utf-8", errors="replace").strip()
        if msg.data:
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScintillatorRawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()