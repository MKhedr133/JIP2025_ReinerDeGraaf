# Radiation Detection Robot (TU Delft JIP)

Autonomous navigation + radiation sensing robot built with **ROS 2 Humble**.  
Includes Nav2, SLAM, and drivers for sensors like the LB-124 scintillator.

## Quickstart

```bash
cd ros2_ws
rosdep install --from-paths src -y --rosdistro humble --skip-keys=ament_python
colcon build --symlink-install

#install pip dependencies
#keyboard-listener node uses pynput dependency. ros2 uses base python when running.
#if you want to use a .venv, you can do the following:
#after colcon build, go to ros2_ws/install/keyboard_listener/lib/keyboard_listener and change the shebang to your preferred environment

source install/setup.bash

#run everything with
ros2 launch simba_bringup simba_launch.py
```

## Keyboard controls
w a s d for tank-control style movement
r and f to increase/decrease linear speed
t and g to increase/decrease angular speed
y to set roomba to "EXPERIMENT" state. Some metrics will be counted and displayed in console in this state (at the moment: /odom and /wheel_tick data)
h to set roomba to "IDLE" state
m to toggle manual controls (ON by default)

## ROS Topic list
Defined in keyboard-listener:
Topic: '/key_input', msg = std_msgs.msg.String, contains key pressed on keyboard

Defined in main-controller:
Topic: '/manual_control_enabled', msg = std_msgs.msg.Bool, flag for enabling keyboard-listener readout

## LIDAR + EKF + SLAM Setup (Current Working Stage)
1. Build & source
```bash
colcon build
source install/setup.bash
```
2. Start sensors (RPLIDAR + static TF)
```bash
ros2 launch create3_lidar_slam sensors_launch.py \
  serial_port:=/dev/ttyUSB0 serial_baudrate:=115200 \
  laser_frame:=laser_frame x:=-0.012 y:=0.0 z:=0.144
```bash
### Checks
```bash
ros2 topic echo --once /scan --qos-reliability best_effort --qos-durability volatile
```
3. Start EKF (odom + IMU → /odometry/filtered)
Fuses /odom and /imu and publishes odom → base_link and /odometry/filtered. 
```bash
ros2 launch create3_lidar_slam ekf_launch.py
```
### Checks
```bash
ros2 topic hz /odometry/filtered            # ~30–50 Hz
ros2 run tf2_tools view_frames              # writes frames.pdf (no GUI in minimal containers)
```
4. Run SLAM mapping (SLAM Toolbox, mapping mode)
Use the all-in-one mapping launch (sensors + EKF + SLAM + RViz).
```bash
ros2 launch create3_lidar_slam full_slam_setup.launch.py
```
If the hardware is on the robot, drive with keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### What you should see in RViz
- LaserScan on /scan
- Growing occupancy Map on /map
- Stable TF: map → odom → base_link → laser_frame

5. Save the map (for Nav2) and the pose-graph
```bash
scripts/save_map_and_graph.sh # create3_lidar_slam
```
