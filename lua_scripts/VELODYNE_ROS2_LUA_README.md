# Velodyne HDL-32E ROS2 Lua Implementation

This is a pure Lua implementation of the Velodyne HDL-32E LiDAR sensor for CoppeliaSim with ROS2 integration, eliminating the need for the compiled C++ plugin.

## Overview

The Velodyne HDL-32E sensor consists of:
- **4 vision sensors** positioned at different angles (-45°, 45°, 135°, -135°)
- **32 laser beams** per sensor
- **Rotating mechanism** that completes ~10 revolutions per second
- **ROS2 publisher** for PointCloud2 messages

## Files

1. **`velodyne_main_ros2.lua`** - Main script attached to the Velodyne base model
2. **`velodyne_sensor_script.lua`** - Vision sensor customization script (for each of the 4 sensors)
3. **`velodyne_ros2_lua.lua`** - Alternative comprehensive implementation (optional)

## Setup Instructions

### Method 1: Using Separate Scripts (Recommended)

#### Step 1: Attach Main Script
1. In CoppeliaSim, select the **Velodyne base model** (the parent object)
2. Right-click → **Add → Associated child script → Non-threaded**
3. Copy the contents of `velodyne_main_ros2.lua` into the script editor
4. Save

#### Step 2: Attach Sensor Scripts
For **each of the 4 vision sensors**:
1. Select the vision sensor object (e.g., `sensor[0]`, `sensor[1]`, etc.)
2. Right-click → **Add → Associated customization script**
3. Copy the contents of `velodyne_sensor_script.lua` into the script editor
4. Save
5. Repeat for all 4 sensors

### Method 2: Using Single Comprehensive Script

1. Attach `velodyne_ros2_lua.lua` to the Velodyne base model
2. Make sure each vision sensor has explicit handling enabled
3. The script will manage everything automatically

## Requirements

- **CoppeliaSim** 4.3.0 or later
- **ROS2 Humble** or later
- **simROS2 plugin** loaded in CoppeliaSim
- Vision sensors must be configured with:
  - Explicit handling enabled
  - Depth buffer output enabled

## Configuration

### Adjustable Parameters (in `velodyne_main_ros2.lua`):

```lua
frequency = 10  -- Rotation frequency in Hz (adjustable: 5-20 Hz)
```

### Vision Sensor Parameters (in sensor scripts):

```lua
{1000, 32}  -- [points per revolution, number of lasers]
0.467748    -- Vertical FOV in radians (~26.8 degrees)
```

## ROS2 Topic

The point cloud is published to:
- **Topic**: `/velodyne_points`
- **Message Type**: `sensor_msgs/msg/PointCloud2`
- **Frame ID**: `velodyne`
- **Fields**: `x`, `y`, `z`, `intensity`

## How It Works

1. **Rotation**: The main script rotates the Velodyne joint continuously
2. **Data Collection**: Each vision sensor processes its depth data via `sysCall_vision`
3. **Packet Generation**: `simVision.velodyneDataFromWorkImg()` converts depth data to Velodyne format
4. **Accumulation**: Packets from all 4 sensors are accumulated per revolution
5. **Publishing**: When a full 360° revolution completes, all packets are combined and published

## Viewing the Point Cloud

### In RViz2:
```bash
ros2 run rviz2 rviz2
```

Add a PointCloud2 display:
- Topic: `/velodyne_points`
- Fixed Frame: `velodyne` or `map`
- Size: 0.01 - 0.05
- Style: Points or Flat Squares

### Verify Messages:
```bash
# Check topic
ros2 topic list | grep velodyne

# Echo messages
ros2 topic echo /velodyne_points

# Check message rate
ros2 topic hz /velodyne_points
```

## Troubleshooting

### No Point Cloud Published
- Verify simROS2 plugin is loaded: **Help → About → Loaded plugins**
- Check that all 4 vision sensors have customization scripts
- Enable explicit handling for vision sensors
- Check CoppeliaSim console for error messages

### ROS2 Context Already Initialized Error
Add this check at the start of each script:
```lua
if not simROS2 then
    local success, result = pcall(require, 'simROS2')
    if success then
        simROS2 = result
    end
end
```

### Low Point Density
- Increase `points_per_revolution` parameter (e.g., 2000 instead of 1000)
- Decrease rotation frequency
- Check vision sensor far clipping plane

### Points in Wrong Coordinate Frame
- Verify `frame_id` in PointCloud2 message matches your TF tree
- Check sensor transformation in CoppeliaSim scene hierarchy
- Ensure base model coordinate frame is correct

## Performance Tips

1. **Reduce point density** if simulation is slow:
   ```lua
   {500, 32}  -- Fewer points per revolution
   ```

2. **Disable visualization** in CoppeliaSim to improve performance

3. **Adjust publish rate**:
   ```lua
   frequency = 5  -- Lower rotation speed
   ```

## Comparison with C++ Plugin

| Feature | C++ Plugin | Lua Implementation |
|---------|-----------|-------------------|
| Installation | Requires compilation | No compilation needed |
| Dependencies | ROS2 C++ libraries | Only simROS2 plugin |
| Customization | Requires rebuilding | Edit scripts directly |
| Performance | Slightly faster | Adequate for most cases |
| Debugging | More difficult | Easier with logs |

## License

This implementation is based on the ITVRoC coppeliasim_plugin_velodyne project, adapted for pure Lua and ROS2.

## References

- [CoppeliaSim simVision API](https://www.coppeliarobotics.com/helpFiles/en/simVision.htm)
- [ROS2 sensor_msgs/PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)
- [Velodyne HDL-32E Specifications](https://velodynelidar.com/products/hdl-32e/)
