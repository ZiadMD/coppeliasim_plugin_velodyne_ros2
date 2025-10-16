# ROS2 Migration Guide

This document describes the changes made to migrate the CoppeliaSim Velodyne plugin from ROS1 to ROS2.

## Major Changes

### 1. Build System
- **Old**: catkin (ROS1)
- **New**: ament_cmake (ROS2)

### 2. Package Configuration (`package.xml`)
- Updated package format to version 3
- Changed `buildtool_depend` from `catkin` to `ament_cmake`
- Replaced `roscpp` with `rclcpp`
- Changed dependency tags from `build_depend`/`run_depend` to `depend`
- Added `<build_type>ament_cmake</build_type>` export

### 3. CMakeLists.txt
- Updated minimum CMake version to 3.8
- Set C++14 as minimum standard
- Replaced `find_package(catkin ...)` with individual `find_package()` calls for ROS2 packages
- Replaced `catkin_package()` with `ament_package()`
- Changed from `include_directories(${catkin_INCLUDE_DIRS})` to `ament_target_dependencies()`
- Removed hardcoded ROS distribution link directories
- Added install rules for ROS2 compatibility
- Changed library type to SHARED explicitly

### 4. Code Changes

#### ros_server_velodyne.h
- Changed `#include <ros/ros.h>` to `#include <rclcpp/rclcpp.hpp>`
- Changed `sensor_msgs::PointCloud2` to `sensor_msgs::msg::PointCloud2`
- Replaced `ros::NodeHandle*` with `rclcpp::Node::SharedPtr`
- Replaced `ros::Publisher` with `rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr`
- Added `getNode()` method to access the node

#### ros_server_velodyne.cpp
- Changed `ros::init()` to `rclcpp::init()`
- Removed `ros::master::check()` (not needed in ROS2)
- Changed `new ros::NodeHandle()` to `rclcpp::Node::make_shared()`
- Changed `node->advertise<>()` to `node->create_publisher<>()`
- Updated QoS from queue size 1 to 10 (ROS2 default)
- Changed shutdown logic to use `rclcpp::shutdown()`
- Updated publisher to use `.reset()` instead of `.shutdown()`
- Added check for `rclcpp::ok()` before initialization and shutdown

#### velodyneROSModel.h
- Changed `#include <ros/ros.h>` to `#include <rclcpp/rclcpp.hpp>`
- Updated message includes to use `msg::` namespace
- Changed publisher type to `rclcpp::Publisher<>::SharedPtr`
- Updated `addPointsToBuffer()` signature to use `sensor_msgs::msg::PointCloud2`

#### velodyneROSModel.cpp
- Updated PointField datatype enum to `sensor_msgs::msg::PointField::FLOAT32`
- Changed `ros::Time::now()` to `node->now()` with null check
- Changed publisher call from `.publish()` method to `->publish()` (pointer)
- Added ROS_server::getNode() to get current time

#### v_repExtVelodyneROS.cpp
- Updated error message to reflect ROS2 terminology

## Building the Plugin

### Prerequisites
- ROS2 (tested with Humble/Iron/Jazzy or later)
- CoppeliaSim (V-REP)
- colcon build tool

### Build Instructions

```bash
# Source your ROS2 workspace
source /opt/ros/<distro>/setup.bash

# Navigate to your workspace
cd /path/to/your/workspace

# Build the package
colcon build --packages-select coppeliasim_plugin_velodyne

# Source the workspace
source install/setup.bash
```

## API Changes Summary

| ROS1 | ROS2 |
|------|------|
| `ros::init()` | `rclcpp::init()` |
| `ros::NodeHandle` | `rclcpp::Node` |
| `ros::Publisher` | `rclcpp::Publisher<T>::SharedPtr` |
| `ros::Time::now()` | `node->now()` |
| `advertise<T>(topic, queue)` | `create_publisher<T>(topic, qos)` |
| `.publish(msg)` | `->publish(msg)` |
| `ros::shutdown()` | `rclcpp::shutdown()` |
| `sensor_msgs::PointCloud2` | `sensor_msgs::msg::PointCloud2` |
| `sensor_msgs::PointField::FLOAT32` | `sensor_msgs::msg::PointField::FLOAT32` |

## Known Issues and Limitations

1. **Spinning**: ROS2 nodes need to be spun to process callbacks. If you're using subscribers or services in the future, you may need to add `rclcpp::spin_some(node)` in the main loop.

2. **QoS Settings**: The default QoS profile is used. You may need to adjust this based on your network requirements.

3. **TF2**: If you need TF transformations, you'll need to migrate from `tf` to `tf2`.

## Testing

After building, test the plugin by:
1. Loading CoppeliaSim with the plugin
2. Verifying the node starts: `ros2 node list`
3. Checking the topic: `ros2 topic list`
4. Echoing point cloud data: `ros2 topic echo /velodyne/points2`

## Future Improvements

- Add parameter server support using ROS2 parameters
- Implement proper lifecycle node management
- Add service/action support if needed
- Consider using composable nodes for better performance
