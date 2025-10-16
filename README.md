# CoppeliaSim Velodyne Plugin for ROS 2

> ⚠️ **WARNING: EXPERIMENTAL - NOT WORKING**  
> This project is currently in an experimental state and **does not work** at the moment. The plugin may cause crashes with the simROS2 plugin in CoppeliaSim. Use at your own risk and expect issues. Contributions and fixes are welcome!

## Overview

This CoppeliaSim plugin publishes Velodyne LiDAR point cloud data to ROS 2 in PointCloud2 format. The plugin is designed to work with CoppeliaSim's built-in Velodyne models (VPL-16, HDL-32E, etc.) and uses C++ for high-performance data serialization and publishing.

### Key Features

- **ROS 2 Native**: Built with `rclcpp` and `ament_cmake`
- **High Performance**: C++ implementation for fast point cloud processing
- **Global Frame Publishing**: Points are published relative to the odom frame with automatic motion compensation
- **Multiple Sensor Support**: Works with VPL-16, HDL-32E, and similar models
- **Compatible**: CoppeliaSim 4.7.0+ and ROS 2 (Humble/Iron/Jazzy)

### What's New in ROS 2

This plugin has been migrated from ROS 1 to ROS 2 with the following changes:
- Build system: `ament_cmake` (instead of catkin)
- ROS client library: `rclcpp` (instead of roscpp)
- Message namespaces: `sensor_msgs::msg::PointCloud2`
- Modern C++14 standards
- QoS-based publisher configuration

## Installation

### Prerequisites

- ROS 2 (Humble, Iron, Jazzy, or later)
- CoppeliaSim 4.7.0+
- colcon build tools
- C++ compiler with C++14 support

### Build Instructions

1. **Clone the repository** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src  # or your workspace directory
   git clone https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2.git coppeliasim_plugin_velodyne
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select coppeliasim_plugin_velodyne
   source install/setup.bash  # or setup.zsh for zsh users
   ```

4. **Copy the plugin to CoppeliaSim**:
   ```bash
   cp ~/ros2_ws/install/coppeliasim_plugin_velodyne/lib/libv_repExtRosVelodyne.so $COPPELIASIM_ROOT_DIR
   ```

### Troubleshooting Build Issues

If you encounter build errors:
```bash
# Clean and rebuild
colcon build --packages-select coppeliasim_plugin_velodyne --cmake-clean-cache
```

## Configuration

### Plugin Configuration

- **Topic Name**: Edit `src/ros_server_velodyne.cpp` to change the published topic. Default: `/velodyne/points2`
- **Frame ID**: Edit `src/velodyneROSModel.cpp` to set the TF frame. Default: `os1_sensor`
- **QoS Settings**: Modify publisher QoS in `ros_server_velodyne.cpp` (default depth: 10)

### CoppeliaSim Scene Setup

The Velodyne sensor in CoppeliaSim requires:
1. **Base Model** - Main Velodyne object with rotating joint
2. **4 Vision Sensors** - Individual sensors at different angles
3. **Scripts** - Lua scripts to interface with the plugin

#### Using the C++ Plugin (Legacy)

For the C++ plugin, use this Lua configuration in your Velodyne model script:

```lua
if (sim_call_type==sim.syscb_init) then
    local visionSensorHandles={}
    for i=1,4,1 do
        visionSensorHandles[i]=sim.getObjectHandle('velodyneVPL_16_sensor'..i)
    end
    local frequency=5 -- 5 Hz
    local options=2+8 -- bit0 (1)=do not display points, 
                -- bit1 (2)=display only current points,
                -- bit2 (4)=returned data is polar (otherwise Cartesian), 
                -- bit3 (8)=displayed points are emissive
    local pointSize=2
    local coloring_closeAndFarDistance={1,4}
    local displayScaling=0.999

    h_velodyne_sensor = sim.getObjectHandle('velodyneVPL_16')

    _h=simExtVelodyneROS_createVelodyneROSModel(visionSensorHandles,frequency,options,pointSize,coloring_closeAndFarDistance,displayScaling, h_velodyne_sensor)
end

if (sim_call_type==sim.syscb_sensing) then
    local fullRev=simExtVelodyneROS_handleVelodyneROSModel(_h, sim.getSimulationTimeStep())
end

if (sim_call_type==sim.syscb_cleanup) then
    simExtVelodyneROS_destroyVelodyneROSModel(_h)
end
```

#### Using ROS 2 Lua Scripts (Experimental)

Alternative Lua scripts for direct ROS 2 integration are available in `/lua_scripts`:
- `velodyne_main_ros2.lua` - Attach to Velodyne base model
- `velodyne_sensor_script.lua` - Attach to each vision sensor

**See `/examples` folder for detailed setup instructions.**

## Usage

### Starting CoppeliaSim with ROS 2

**Important**: Source your ROS 2 environment before starting CoppeliaSim:

```bash
source /opt/ros/humble/setup.bash  # or your ROS 2 distro
source ~/ros2_ws/install/setup.bash
cd $COPPELIASIM_ROOT_DIR
./coppeliaSim.sh
```

### Loading a Scene

1. Open CoppeliaSim
2. Load a scene containing a Velodyne sensor model
3. Ensure the plugin loaded successfully (check console for messages)
4. Start the simulation

### Monitoring ROS 2 Topics

```bash
# List all topics
ros2 topic list

# Check topic info
ros2 topic info /velodyne/points2

# Echo point cloud data (may be large!)
ros2 topic echo /velodyne/points2 --max-count 1

# Check publish rate
ros2 topic hz /velodyne/points2
```

### Visualizing in RViz2

```bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Set **Fixed Frame** to `velodyne` or `odom`
2. Add **PointCloud2** display
3. Set **Topic** to `/velodyne/points2`
4. Adjust point size and color as needed

### Verifying the Node

```bash
# List running nodes
ros2 node list
# Should show: /vrep_velodyne

# Check node info
ros2 node info /vrep_velodyne
```

## API Reference

### Lua API Functions

The plugin provides three main functions callable from CoppeliaSim Lua scripts:

#### `simExtVelodyneROS_createVelodyneROSModel()`

Creates and initializes a Velodyne ROS model.

```lua
handle = simExtVelodyneROS_createVelodyneROSModel(
    visionSensorHandles,  -- table of 4 vision sensor handles
    frequency,            -- rotation frequency in Hz
    options,              -- display options (bitmask)
    pointSize,            -- point size for visualization
    coloringDistances,    -- {near, far} distances for color coding
    displayScaling,       -- scaling factor (typically 0.999)
    localFrameHandle      -- handle to local reference frame
)
```

**Returns**: Integer handle to the Velodyne model, or -1 on failure.

#### `simExtVelodyneROS_handleVelodyneROSModel()`

Processes sensor data and publishes point cloud. Call this in `sysCall_sensing()`.

```lua
revolutionComplete = simExtVelodyneROS_handleVelodyneROSModel(
    handle,  -- Velodyne model handle
    dt       -- simulation time step
)
```

**Returns**: Boolean - true if a full revolution was completed.

#### `simExtVelodyneROS_destroyVelodyneROSModel()`

Cleans up and destroys the Velodyne model.

```lua
simExtVelodyneROS_destroyVelodyneROSModel(handle)
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/velodyne/points2` | `sensor_msgs/msg/PointCloud2` | Point cloud data from complete revolution |

### PointCloud2 Message Details

- **Frame ID**: Configurable (default: `os1_sensor` or `velodyne`)
- **Fields**: x, y, z, intensity (FLOAT32)
- **Publishing**: Once per complete revolution
- **Coordinate Frame**: Global (odom frame) with motion compensation

## ROS 2 Migration Notes

This plugin was migrated from ROS 1. Key API changes:

| ROS 1 | ROS 2 |
|-------|-------|
| `ros::init()` | `rclcpp::init()` |
| `ros::NodeHandle` | `rclcpp::Node::SharedPtr` |
| `ros::Publisher` | `rclcpp::Publisher<T>::SharedPtr` |
| `ros::Time::now()` | `node->now()` |
| `advertise<T>(topic, queue)` | `create_publisher<T>(topic, qos)` |
| `.publish(msg)` | `->publish(msg)` |
| `sensor_msgs::PointCloud2` | `sensor_msgs::msg::PointCloud2` |

## Troubleshooting

### Plugin Doesn't Load

**Symptoms**: No plugin messages in CoppeliaSim console

**Solutions**:
- Verify ROS 2 environment is sourced before starting CoppeliaSim
- Check plugin file is in `$COPPELIASIM_ROOT_DIR`
- Ensure correct library name: `libv_repExtRosVelodyne.so`
- Check CoppeliaSim console for error messages

### No Topics Published

**Symptoms**: `ros2 topic list` doesn't show `/velodyne/points2`

**Solutions**:
- Check if node is running: `ros2 node list`
- Verify Velodyne model was created (handle != -1)
- Ensure simulation is running in CoppeliaSim
- Check vision sensors are properly configured

### CoppeliaSim Crashes

**Symptoms**: Application crashes during simulation

**Solutions**:
- **KNOWN ISSUE**: May conflict with simROS2 plugin
- Try reducing rotation frequency
- Check for memory leaks
- Update to latest CoppeliaSim version (4.7.0+)
- See `/examples/troubleshooting.md` for more details

### Empty Point Clouds

**Symptoms**: Topic publishes but no points visible in RViz2

**Solutions**:
- Check vision sensor settings (near/far clipping)
- Verify objects are in sensor range
- Check frame transforms in RViz2
- Ensure correct TF frame is set

### Build Errors

**Symptoms**: Compilation fails

**Solutions**:
```bash
# Clean build
colcon build --packages-select coppeliasim_plugin_velodyne --cmake-clean-cache

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y

# Verify ROS 2 is sourced
echo $ROS_DISTRO
```

## Project Structure

```
coppeliasim_plugin_velodyne/
├── README.md                    # This file
├── CMakeLists.txt              # Build configuration
├── package.xml                 # ROS 2 package manifest
├── include/                    # Header files
│   └── coppeliasim_plugin_velodyne/
│       ├── ros_server_velodyne.h
│       └── velodyneROSModel.h
├── src/                        # Source files
│   ├── ros_server_velodyne.cpp
│   ├── velodyneROSModel.cpp
│   └── v_repExtVelodyneROS.cpp
├── lua_scripts/                # Lua scripts for CoppeliaSim
│   ├── README.md
│   ├── velodyne_main_ros2.lua
│   └── velodyne_sensor_script.lua
└── examples/                   # Usage examples and guides
    ├── README.md
    ├── sample_configuration.lua
    ├── alternative_approach.lua
    └── troubleshooting.md
```

## TODO

- [ ] **Critical**: Fix compatibility with simROS2 plugin to prevent crashes
- [ ] Test with latest CoppeliaSim versions (4.7.x+)
- [ ] Verify ROS 2 topic publishing and message format correctness
- [ ] Add parameter server support for runtime configuration
- [ ] Implement proper lifecycle node management
- [ ] Add QoS configuration options
- [ ] Support for different Velodyne models (VLP-16, HDL-64E)
- [ ] Performance optimization and memory leak checks
- [ ] Comprehensive testing suite
- [ ] Documentation improvements and video tutorials

## Known Issues

- ⚠️ **May crash CoppeliaSim** when used with simROS2 plugin
- Point cloud data format may not parse correctly in all cases
- Revolution detection trigger may be unreliable
- Performance degradation with high-frequency rotation
- No parameter server integration yet
- Limited error handling and recovery

## Contributing

Contributions are welcome! If you find bugs or have improvements:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Make your changes
4. Test thoroughly
5. Submit a pull request

Please include:
- Clear description of the problem/improvement
- Test results
- Updated documentation if needed

## License

[Specify your license here]

## Credits

- Original ROS 1 plugin: [ITVRoC/coppeliasim_plugin_velodyne](https://github.com/ITVRoC/coppeliasim_plugin_velodyne)
- ROS 2 migration and improvements: ZiadMD
- CoppeliaSim/V-REP: Coppelia Robotics

## References

- [CoppeliaSim](https://www.coppeliarobotics.com/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)
- [Velodyne LiDAR](https://velodynelidar.com/)

## Support

For issues, questions, or discussions:
- **GitHub Issues**: [Report bugs and request features](https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2/issues)
- **Examples & Guides**: See the `/examples` folder
- **CoppeliaSim Forum**: [forum.coppeliarobotics.com](https://forum.coppeliarobotics.com/)
- **ROS Answers**: [answers.ros.org](https://answers.ros.org/)

---

**Repository**: [https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2](https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2)

**Status**: 🔴 Experimental - Not Working
