# Quick Start Guide - ROS2 Velodyne Plugin

## Successfully Migrated! ✅

The CoppeliaSim Velodyne plugin has been successfully migrated to ROS2.

## What Changed?

This plugin now uses **ROS2** instead of ROS1:
- Build system: `ament_cmake` (instead of catkin)
- ROS client library: `rclcpp` (instead of roscpp)
- Message namespaces: `sensor_msgs::msg::PointCloud2` (instead of sensor_msgs::PointCloud2)

## Build Status

✅ **Build Successful** (with minor warnings about unused parameters)

Library generated: `install/coppeliasim_plugin_velodyne/lib/libv_repExtRosVelodyne.so`

## Usage

### 1. Source your ROS2 workspace
```bash
source /opt/ros/<your-ros2-distro>/setup.bash
source install/setup.bash
```

### 2. Load the plugin in CoppeliaSim
Copy or link the library to your CoppeliaSim installation:
```bash
cp install/coppeliasim_plugin_velodyne/lib/libv_repExtRosVelodyne.so \
   /path/to/coppeliasim/
```

### 3. Monitor ROS2 Topics
```bash
# List topics
ros2 topic list

# Echo point cloud data
ros2 topic echo /velodyne/points2

# Check topic info
ros2 topic info /velodyne/points2
```

### 4. Verify Node
```bash
# List running nodes
ros2 node list

# Should show: /vrep_velodyne
```

## Topics Published

- **`/velodyne/points2`** (`sensor_msgs/msg/PointCloud2`)
  - Point cloud data from the Velodyne sensor
  - Published once per complete revolution
  - Frame ID: `velodyne` (or configured frame)

## Lua API (unchanged)

The Lua API in CoppeliaSim scripts remains the same:

```lua
-- Create velodyne model
velodyneHandle = simExtVelodyneROS_createVelodyneROSModel(
    visionSensorHandles,  -- table of 4 vision sensors
    frequency,            -- scan frequency
    options,             -- display options
    pointSize,           -- point size
    coloringDistances,   -- near/far distances for coloring
    displayScalingFactor, -- scaling factor
    LocalFrameHandle     -- local reference frame
)

-- Handle velodyne in simulation step
result = simExtVelodyneROS_handleVelodyneROSModel(velodyneHandle, dt)

-- Destroy when done
simExtVelodyneROS_destroyVelodyneROSModel(velodyneHandle)
```

## Troubleshooting

### Plugin doesn't load
- Verify ROS2 environment is sourced before starting CoppeliaSim
- Check CoppeliaSim console for error messages
- Ensure the library is in CoppeliaSim's plugin directory

### No topics appearing
- Check if the node is running: `ros2 node list`
- Verify sensor configuration in your scene
- Ensure the velodyne handle was created successfully (returns valid handle, not -1)

### Build issues
```bash
# Clean and rebuild
colcon build --packages-select coppeliasim_plugin_velodyne --cmake-clean-cache
```

## Next Steps

- Review `ROS2_MIGRATION.md` for detailed migration information
- Test with your CoppeliaSim scene
- Adjust QoS settings if needed for your network
- Consider adding parameter support for runtime configuration

## Support

For issues or questions:
1. Check the migration guide: `ROS2_MIGRATION.md`
2. Review ROS2 documentation: https://docs.ros.org/
3. Check CoppeliaSim forums: https://forum.coppeliarobotics.com/
