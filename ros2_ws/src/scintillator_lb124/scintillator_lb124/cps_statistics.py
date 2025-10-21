import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


import numpy


class StatisticsCPSNode(Node):
    def __init__(self):
        super().__init__('cps_listener')
        self.subscription = self.create_subscription(
            Float32,
            'scintillator/cps',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publishers for statistics
        self.mean_publisher = self.create_publisher(Float32, 'scintillator/cps_mean', 10)
        self.std_dev_publisher = self.create_publisher(Float32, 'scintillator/cps_std_dev', 10)
        self.median_publisher = self.create_publisher(Float32, 'scintillator/cps_median', 10)
        

        # Hold last number of cps values. This will be used to publish statistics
        self.window_size = 10
        self.window = numpy.zeros(self.window_size)
        self.index = 0

        # Timer to publish statistics periodically
        self.timer = self.create_timer(0.5, self.publish_statistics)
        
        
    def listener_callback(self, msg):
        if numpy.isnan(msg.data):   #prevent errors from NaN values
            return
        # update value at current index
        self.window[self.index % self.window_size] = msg.data
        self.index += 1

    def publish_statistics(self):
        # Calculate statistics on the current window of data
        mean = numpy.mean(self.window)
        std_dev = numpy.std(self.window)
        median = numpy.median(self.window)

        self.mean_publisher.publish(Float32(data=mean))
        self.std_dev_publisher.publish(Float32(data=std_dev))
        self.median_publisher.publish(Float32(data=median))

def main(args=None):
    rclpy.init(args=args)
    statisticsNode = StatisticsCPSNode()
    rclpy.spin(statisticsNode)

    statisticsNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()