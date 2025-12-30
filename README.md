# rplidar_utilities

A ROS2 node with utility functionalities for a SLAMTEC LIDAR.

Provides:

- the node 'rplidar_info' to display information about the RPLIDAR
- the node 'rplidar_listener' to listen to/display the LaserScan messages
- a rviz config-file to display the LaserScan messages (with topic name 'scan') in RVIZ

The nodes use the 'rplidar_sdk_ros2' package, which provides the public SDK of RPLIDAR products in C++ as a ROS package.

## Run rplidar_info

```bash
ros2 run rplidar_utilities rplidar_info [parameters ...]
```

Or with the launch-file (for a RPLIDAR C1):

```bash
ros2 launch rplidar_utilities rplidar_info_c1_launch.py
```

Node parameters:

- channel_type (default = 'serial')
- serial_port (default = '/dev/ttyUSB0')
- serial_baudrate (default = '460800')

## Run rplidar_listener

```bash
ros2 run rplidar_utilities rplidar_listener
```

Or

```bash
ros2 launch rplidar_utilities rplidar_listener_launch.py
```

Node parameter:

- topic_name  (default = 'scan')

## View 'scan' messages in rviz

With the launch-file:

```bash
ros2 launch rplidar_rviz_launch.py
```

Or start RVIZ directly

```bash
ros2 run rviz rviz
```

---

![C++17](https://img.shields.io/badge/C++-17-green)
![License](https://img.shields.io/badge/Apache--2.0-orange)

Tested with:

![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![RPLIDAR C1](https://img.shields.io/badge/RPLIDAR--C1-green)

---
