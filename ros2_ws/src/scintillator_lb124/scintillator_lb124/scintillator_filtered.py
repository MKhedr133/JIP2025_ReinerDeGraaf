#!/usr/bin/env python3

"""
Raw -> CPS extraction + threshold
===============================
Subscribes to `/scintillator/raw`, extracts CPS numbers, and publishes:
- `/scintillator/cps` (std_msgs/Float32)
- `/scintillator/status` (std_msgs/String: "OK" or "ALERT")

Parameters
- threshold (double, default 20.0) — alert threshold
- cps_index (int, default 1) — if multiple CPS numbers appear, which one to use (0-based)
"""

from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


def extract_cps_all(line: str) -> List[float]:
    parts = line.split()
    out = []
    for i, tok in enumerate(parts):
        if tok.lower() == "cps" and i > 0:
            try:
                out.append(float(parts[i - 1]))
            except ValueError:
                pass
    return out


class ScintillatorFilterNode(Node):
    def __init__(self):
        super().__init__("scintillator_filter")
        self.declare_parameter("threshold", 50.0)
        self.declare_parameter("cps_index", 1)
        self.threshold = float(self.get_parameter("threshold").value)
        self.cps_index = int(self.get_parameter("cps_index").value)

        self.pub_cps = self.create_publisher(Float32, "/scintillator/cps", 10)
        self.pub_status = self.create_publisher(String, "/scintillator/status", 10)
        self.create_subscription(String, "/scintillator/raw", self.on_raw, 10)
        self.get_logger().info("Listening on /scintillator/raw -> publishing CPS & status")

    def on_raw(self, msg: String):
        vals = extract_cps_all(msg.data)
        if not vals:
            return
        idx = self.cps_index if 0 <= self.cps_index < len(vals) else -1
        cps = float(vals[idx])

        cps_msg = Float32(); cps_msg.data = cps
        self.pub_cps.publish(cps_msg)

        st = String(); st.data = "OK" if cps < self.threshold else "ALERT"
        self.pub_status.publish(st)

        self.get_logger().info(f"CPS={cps:.2f}  Status={st.data}")

def main():
    rclpy.init()
    node = ScintillatorFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
