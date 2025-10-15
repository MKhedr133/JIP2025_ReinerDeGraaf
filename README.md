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

## LIDAR + SLAM Setup (Current Working Stage)
1. Run the full mapping stack:
```bash
ros2 launch create3_lidar_slam full_slam_setup.launch.py
```

2. Afterwards if hardware is mounted to the roomba, run teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
``` 

3. Drive around and watch the map update in RViz
4. Save map for Nav2:
```bash
ros2 run nav2_map_server map_saver_cli -f src/maps/create3_map
```