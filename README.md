# HUSARION ROSBOT UGV GUI

This repository contains the ROS2 humble workspace for a HUSARION ugv. It includes:

- A sleek PyQt5 GUI for controlling the rover and visualizing camera & battery data.
- Nodes for publishing and subscribing to `/cmd_vel`, `/camera/image_raw`, and `/battery/battery_status`.
- Integrated Gazebo and Rviz2 simulation and integration with the Husarion ugv bot.
- Keyboard and GUI control for movement and battery monitoring.

## Getting Started

```
git clone https://github.com/wakifrajin/rosbot_ugv_gui.git
cd erc_ws
colcon build
source install/setup.bash
```

## Run GUI
```
ros2 run erc_gui gui_node
```

## Dependencies

ROS2 Humble

rclpy

PyQt5

OpenCV (cv2)

cv_bridge

yaml
