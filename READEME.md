## Ros commands

### Start vnc graphic display server

vncserver -kill :3 2>/dev/null || true
vncserver :3 -localhost no -geometry 1280x800 -depth 24

vncserver -kill :4 2>/dev/null || true
vncserver :4 -localhost no -geometry 1280x800 -depth 24

### Ros build 

cd ~/ros2_ws
colcon build --symlink-install --packages-select mlx90640_ros2

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

### Run thermal nodes


ros2 run mlx90640_ros2 mlx90640_serial

ros2 run mlx90640_ros2 thermal_visualizer




