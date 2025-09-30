# Radiation Detection Robot (TU Delft JIP)

Autonomous navigation + radiation sensing robot built with **ROS 2 Humble**.  
Includes Nav2, SLAM, and drivers for sensors like the LB-124 scintillator.

## Quickstart

```bash
cd ros2_ws
rosdep install --from-paths src -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```