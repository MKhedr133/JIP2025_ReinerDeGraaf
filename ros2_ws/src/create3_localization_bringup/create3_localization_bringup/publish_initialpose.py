#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter

def yaw_to_quat(z):
    return (0.0, 0.0, math.sin(z/2.0), math.cos(z/2.0))

class InitialPosePub(Node):
    def __init__(self):
        super().__init__('initial_pose_pub')
        # Declare params (meters, radians)
        self.declare_parameter('x', 0.10)
        self.declare_parameter('y', 0.05)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('delay_sec', 2.0)

        self.timer = self.create_timer(
            self.get_parameter('delay_sec').value, self.publish_once)

    def publish_once(self):
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        frame_id = self.get_parameter('frame_id').value
        qx, qy, qz, qw = yaw_to_quat(yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # small covariance to avoid overconfidence
        msg.pose.covariance[0] = 0.25    # x
        msg.pose.covariance[7] = 0.25    # y
        msg.pose.covariance[35] = 0.0685 # yaw (~15 deg^2)

        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        pub.publish(msg)
        self.get_logger().info(f"Published /initialpose at ({x:.2f}, {y:.2f}, yaw {yaw:.2f} rad) in {frame_id}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = InitialPosePub()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
