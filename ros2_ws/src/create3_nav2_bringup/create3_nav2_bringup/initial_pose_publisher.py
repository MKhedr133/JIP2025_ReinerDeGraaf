#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(2.0, self.publish_once)

    def publish_once(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.get_logger().info('Published initial pose.')
        time.sleep(1)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
