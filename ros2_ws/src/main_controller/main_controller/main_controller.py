"""
Main Keyboard-Driven State Controller

This ROS 2 node acts as the primary user interface for controlling the robot's
high-level operational state via keyboard commands. It listens for specific key
presses and publishes the corresponding state to the `/simba_state` topic, which
is then used by other nodes (like the `autonomous_controller`) to trigger
specific behaviors.

### Functionality:
- **State Management**: Translates keyboard inputs into system-wide states.
  - 'm': Toggles between `IDLE` and `DRIVE` (to start/stop autonomous behavior).
  - 'y': Enters `EXPERIMENT` mode.
  - 'h': Returns to `IDLE` mode, stopping all activities.
- **Odometry Tracking**: In the `EXPERIMENT` state, it uses wheel tick data to
  calculate and log the distance traveled.
- **Safety Override**: On startup, it calls a service to disable the robot's
  built-in safety limits, allowing for unrestricted movement (e.g., driving backward).

### Subscribed Topics:
- `/key_input` (std_msgs/String): Receives keyboard commands.
- `/wheel_ticks` (irobot_create_msgs/msg/WheelTicks): Used for distance tracking.

### Published Topics:
- `/simba_state` (std_msgs/String): Broadcasts the current system state (e.g., "IDLE", "DRIVE").
- `/manual_control_enabled` (std_msgs/Bool): Publishes a flag for manual control status (currently static).

### States:
- `IDLE`: The default state where the robot is stationary.
- `DRIVE`: A state that signals the autonomous controller to begin its routine.
- `EXPERIMENT`: A state for running specific tests, such as distance measurement.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import WheelTicks
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math


from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue 
from rcl_interfaces.msg import Parameter


class MainControllerNode(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Initial state
        self.state = 'IDLE'
        self.get_logger().info(f"Initial state: {self.state}")
        self.statePublisher = self.create_publisher(
            String,
            'simba_state',
            10
        )


        # Keep track of Distance Travelled in EXPERIMENT state
        self.startpose = (0.0,0.0,0.0) # TODO: save this in a better data format
        self.startTickLeft = 0
        self.startTickRight = 0
        
        # Subscriber to /key_input
        self.keyboardSubscriber = self.create_subscription(
            String,
            'key_input',
            self.key_input_callback,
            10
        )

        # make sure QOS lines up with roomba stuff
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscriber to /wheel_ticks
        self.tickSubscriber = self.create_subscription(
            WheelTicks,
            'wheel_ticks',
            self.tick_callback,
            qos)
        
        # Publish whether MANUAL control is enabled on /manual_control_enabled
        self.manualControlEnabled = True
        self.manualControlPublisher = self.create_publisher(
            Bool,
            'manual_control_enabled',
            10
        )

        # Turn off safety limits
        self.client = self.create_client(SetParameters, '/motion_control/set_parameters')
        self.turn_off_safety_limits()



    def key_input_callback(self, msg: String):
        key = msg.data.strip()
        self.get_logger().info(f"Received key input: '{key}'")

        if key == 'y':
            self.state = 'EXPERIMENT'
            self.statePublisher.publish(String(data=self.state))
            self.get_logger().info(f"State changed to: {self.state}. Starting experiment.")
        elif key == 'h':
            self.state = 'IDLE'
            self.statePublisher.publish(String(data=self.state))
            self.get_logger().info(f"State changed to: {self.state}. Stopping experiment.")
        elif key == 'm':
            # Toggle modes
            if self.state == 'IDLE':
                self.state = 'DRIVE'
                self.statePublisher.publish(String(data=self.state))
                self.get_logger().info(f"State changed to: {self.state}")
            # elif self.state == 'STOP':
            else:
                self.state = 'IDLE'
                self.statePublisher.publish(String(data=self.state))
                self.get_logger().info(f"State changed to: {self.state}")

        else:
            self.get_logger().info(f"No state change (current state: {self.state})")

    def tick_callback(self, msg: WheelTicks):
        if self.state == 'EXPERIMENT':
            ticks_travelled_avg = ((msg.ticks_left - self.startTickLeft) + (msg.ticks_right - self.startTickRight)) / 2.0
            circumference = math.pi * 72.0  # in mm
            dist_travelled = ticks_travelled_avg / 508.8 * circumference
            # Log wheel ticks
            # self.get_logger().info(f"WHEELTICK distance {dist_travelled / 100.0} cm")
        elif self.state == 'IDLE':
            # Update start ticks to current ticks
            self.startTickLeft = msg.ticks_left
            self.startTickRight = msg.ticks_right

    # source: https://github.com/tuftsceeo/Tufts_Create3_Examples/blob/main/Projects/Penalty_Shootout/penalty_kick.py#L169
    def turn_off_safety_limits(self):
        '''
        Override the safety control paramter in another node so that we can drive backwards.
        '''
        
        '''
        get the request we will send to the service server
        '''
        request = SetParameters.Request()
        
        '''
        edit that paramter message with the correct parameter name & value
        '''
        param = Parameter() 
        param.name = "safety_override"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = 'full'
        '''
        append the service request with the correct parameter message
        '''
        request.parameters.append(param)
        
        '''
        wait until the service server is available then send the request
        '''
        self.client.wait_for_service()
        self.future = self.client.call_async(request)
        self.get_logger().info('Safety limits turned off')

def main(args=None):
    rclpy.init(args=args)
    node = MainControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
