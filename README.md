# SLAM ROS Package

This ROS 2 package is designed for recording and storing robot movements and implementing SLAM functionality.

## Prerequisites

- ROS 2 Galactic
- CMake 3.8 or higher
- C++ compiler with C++14 support

## Dependencies

- rclcpp
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2
- tf2_ros

## Building the Package

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/dongmin0204/SLAM_ROS.git
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select slam_ros
```

3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Recording Robot Movements

To start recording robot movements:

```bash
ros2 launch slam_ros movement_recorder.launch.py
```

The node will subscribe to the `/odom` topic and record the robot's position and orientation to a CSV file named `robot_movements.csv` in the current directory.

The CSV file will contain the following columns:
- timestamp
- x, y, z (position)
- orientation_x, orientation_y, orientation_z, orientation_w (orientation quaternion)

## Future Features

- SLAM implementation
- Map visualization
- Path planning
- Location data storage and retrieval 
