# CoppeliaSim Velodyne Plugin for ROS 2

<div align="center">

![Status](https://img.shields.io/badge/status-experimental-red)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20|%20Iron%20|%20Jazzy-blue)
![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-4.7.0+-green)
![License](https://img.shields.io/badge/license-Unspecified-lightgrey)

</div>

> ⚠️ **WARNING: EXPERIMENTAL - NOT WORKING**  
> This project is currently in an experimental state and **does not work** at the moment. The plugin may cause crashes with the simROS2 plugin in CoppeliaSim. Use at your own risk and expect issues. Contributions and fixes are welcome!

---

## 📋 Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [API Reference](#api-reference)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [Credits](#credits)

---

## 🎯 Overview

This CoppeliaSim plugin enables Velodyne LiDAR point cloud publishing to ROS 2 in PointCloud2 format. Designed for CoppeliaSim's built-in Velodyne models (VPL-16, HDL-32E, etc.), it uses high-performance C++ for efficient data serialization and publishing.

### ✨ Key Features

- **🚀 ROS 2 Native** - Built with `rclcpp` and `ament_cmake`
- **⚡ High Performance** - C++ implementation for fast point cloud processing
- **🌍 Global Frame Publishing** - Points published relative to odom frame with automatic motion compensation
- **🔧 Multiple Sensor Support** - Compatible with VPL-16, HDL-32E, and similar models
- **✅ Modern Standards** - CoppeliaSim 4.7.0+ and ROS 2 (Humble/Iron/Jazzy)

### 🆕 What's New in ROS 2

This plugin has been migrated from ROS 1 to ROS 2 with significant improvements:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Build System | catkin | ament_cmake |
| Client Library | roscpp | rclcpp |
| Message Namespace | sensor_msgs:: | sensor_msgs::msg:: |
| C++ Standard | C++11 | C++14 |
| Communication | Topic-based | QoS-based publishers |

---

## 📦 Installation

### Prerequisites

Ensure you have the following installed:

- **ROS 2** - Humble, Iron, Jazzy, or later ([Installation Guide](https://docs.ros.org/))
- **CoppeliaSim** - Version 4.7.0 or higher ([Download](https://www.coppeliarobotics.com/))
- **colcon** - ROS 2 build tool
- **C++ Compiler** - With C++14 support (GCC 7+, Clang 5+)

### 🔨 Build Instructions

**1. Clone the repository**

```bash
cd ~/ros2_ws/src  # Navigate to your ROS 2 workspace
git clone https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2.git coppeliasim_plugin_velodyne
```

**2. Install dependencies**

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build the package**

```bash
colcon build --packages-select coppeliasim_plugin_velodyne
source install/setup.bash  # For bash users
# source install/setup.zsh  # For zsh users
```

**4. Install the plugin to CoppeliaSim**

```bash
cp ~/ros2_ws/install/coppeliasim_plugin_velodyne/lib/libv_repExtRosVelodyne.so $COPPELIASIM_ROOT_DIR
```

> **💡 Tip**: Set `COPPELIASIM_ROOT_DIR` in your `.bashrc` or `.zshrc`:
> ```bash
> export COPPELIASIM_ROOT_DIR=/path/to/coppeliasim
> ```

### 🔧 Troubleshooting Build Issues

If you encounter build errors, try:

```bash
# Clean build cache and rebuild
colcon build --packages-select coppeliasim_plugin_velodyne --cmake-clean-cache

# Verify ROS 2 environment
echo $ROS_DISTRO  # Should output: humble, iron, jazzy, etc.

# Check for missing dependencies
rosdep check --from-paths src --ignore-src
```

---

## ⚙️ Configuration

### Plugin Parameters

Configure these settings by editing the source files before building:

| Parameter | File | Default | Description |
|-----------|------|---------|-------------|
| Topic Name | `src/ros_server_velodyne.cpp` | `/velodyne/points2` | ROS 2 topic for point cloud |
| Frame ID | `src/velodyneROSModel.cpp` | `os1_sensor` | TF frame for transforms |
| QoS Depth | `src/ros_server_velodyne.cpp` | `10` | Publisher queue size |

### CoppeliaSim Scene Setup

The Velodyne sensor requires specific scene configuration:

#### Required Components

1. **Base Model** - Main Velodyne object with rotating joint
2. **4 Vision Sensors** - Positioned at 0°, 90°, 180°, 270° offsets
3. **Lua Scripts** - Interface scripts for the plugin

#### Option A: Using the C++ Plugin (Recommended for ROS 2)

Add this script to your Velodyne base model in CoppeliaSim:

```lua
function sysCall_init()
    -- Get vision sensor handles
    local visionSensorHandles = {}
    for i = 1, 4 do
        visionSensorHandles[i] = sim.getObjectHandle('velodyneVPL_16_sensor' .. i)
    end
    
    -- Configuration parameters
    local frequency = 5              -- Rotation frequency (Hz)
    local options = 2 + 8            -- Display options bitmask
    local pointSize = 2              -- Visualization point size
    local coloringDistance = {1, 4}  -- Near/far color coding (meters)
    local displayScaling = 0.999     -- Prevents z-fighting
    
    -- Get local frame
    local h_velodyne_sensor = sim.getObjectHandle('velodyneVPL_16')
    
    -- Create Velodyne model
    velodyneHandle = simExtVelodyneROS_createVelodyneROSModel(
        visionSensorHandles,
        frequency,
        options,
        pointSize,
        coloringDistance,
        displayScaling,
        h_velodyne_sensor
    )
end

function sysCall_sensing()
    -- Process and publish data each simulation step
    local fullRev = simExtVelodyneROS_handleVelodyneROSModel(
        velodyneHandle,
        sim.getSimulationTimeStep()
    )
end

function sysCall_cleanup()
    -- Clean up resources
    simExtVelodyneROS_destroyVelodyneROSModel(velodyneHandle)
end
```

#### Option B: Using ROS 2 Lua Scripts (Experimental)

Alternative pure Lua scripts are available in `/lua_scripts/`:
- `velodyne_main_ros2.lua` - Main controller script
- `velodyne_sensor_script.lua` - Vision sensor processing

> 📚 **See [`/examples`](./examples) folder for detailed setup instructions and configurations**

---

## 🚀 Usage

### Starting CoppeliaSim with ROS 2

> ⚠️ **Important**: Always source your ROS 2 environment before starting CoppeliaSim

```bash
# Source ROS 2 distribution
source /opt/ros/humble/setup.bash  # Replace 'humble' with your distro

# Source your workspace
source ~/ros2_ws/install/setup.bash

# Start CoppeliaSim
cd $COPPELIASIM_ROOT_DIR
./coppeliaSim.sh
```

### Quick Start

1. **Open CoppeliaSim** and load a scene with a Velodyne sensor model
2. **Verify plugin loaded** - Check console for "Velodyne" plugin messages
3. **Start simulation** - Click the play button ▶️
4. **Monitor topics** - Use the commands below to verify data flow

### Monitoring ROS 2 Topics

```bash
# List all active topics
ros2 topic list

# View topic information
ros2 topic info /velodyne/points2

# Echo point cloud data (limit to 1 message due to size)
ros2 topic echo /velodyne/points2 --max-count 1

# Monitor publishing frequency
ros2 topic hz /velodyne/points2

# Check message bandwidth
ros2 topic bw /velodyne/points2
```

### Visualizing in RViz2

**Launch RViz2:**
```bash
ros2 run rviz2 rviz2
```

**Configure RViz2:**
1. Set **Fixed Frame** → `velodyne` or `odom`
2. Click **Add** → **By topic** → `/velodyne/points2` → **PointCloud2**
3. Adjust visualization:
   - **Size (m)**: 0.01 - 0.05
   - **Style**: Points or Flat Squares
   - **Color Transformer**: Intensity or AxisColor

### Verifying the ROS 2 Node

```bash
# List running nodes
ros2 node list
# Expected output: /vrep_velodyne

# Get detailed node information
ros2 node info /vrep_velodyne

# View node graph
rqt_graph
```

---

## 📚 API Reference

### Lua API Functions

The plugin exposes three main functions for CoppeliaSim Lua scripts:

#### `simExtVelodyneROS_createVelodyneROSModel()`

Creates and initializes a Velodyne ROS 2 model.

**Syntax:**
```lua
handle = simExtVelodyneROS_createVelodyneROSModel(
    visionSensorHandles,  -- table: Array of 4 vision sensor handles
    frequency,            -- number: Rotation frequency in Hz (5-20)
    options,              -- number: Display options bitmask
    pointSize,            -- number: Point size for visualization (1-10)
    coloringDistances,    -- table: {near, far} distances in meters
    displayScaling,       -- number: Scaling factor (0.95-1.0)
    localFrameHandle      -- number: Handle to local reference frame
)
```

**Parameters:**
- `visionSensorHandles`: Table containing exactly 4 vision sensor handles
- `frequency`: Rotation speed in Hz (typical: 5-10 Hz)
- `options`: Bitmask - `1`=hide points, `2`=current only, `4`=polar coords, `8`=emissive
- `pointSize`: Visual point size (default: 2)
- `coloringDistances`: Distance range for color coding `{near, far}`
- `displayScaling`: Prevents z-fighting (typical: 0.999)
- `localFrameHandle`: Reference frame for coordinate transformations

**Returns:** 
- `number`: Model handle (> 0) on success, `-1` on failure

**Example:**
```lua
local sensors = {sensor1, sensor2, sensor3, sensor4}
local handle = simExtVelodyneROS_createVelodyneROSModel(
    sensors, 10, 10, 2, {1, 4}, 0.999, baseFrame
)
if handle == -1 then
    sim.addLog(sim.verbosity_scripterrors, "Failed to create Velodyne model")
end
```

---

#### `simExtVelodyneROS_handleVelodyneROSModel()`

Processes sensor data and publishes point cloud to ROS 2. Call this in `sysCall_sensing()`.

**Syntax:**
```lua
revolutionComplete = simExtVelodyneROS_handleVelodyneROSModel(
    handle,  -- number: Velodyne model handle from create function
    dt       -- number: Simulation time step (from sim.getSimulationTimeStep())
)
```

**Returns:**
- `boolean`: `true` if a full 360° revolution was completed and data published

**Example:**
```lua
function sysCall_sensing()
    local completed = simExtVelodyneROS_handleVelodyneROSModel(
        velodyneHandle,
        sim.getSimulationTimeStep()
    )
    if completed then
        sim.addLog(sim.verbosity_scriptinfos, "Point cloud published")
    end
end
```

---

#### `simExtVelodyneROS_destroyVelodyneROSModel()`

Cleans up and destroys the Velodyne model. Call this in `sysCall_cleanup()`.

**Syntax:**
```lua
simExtVelodyneROS_destroyVelodyneROSModel(handle)
```

**Parameters:**
- `handle`: Model handle returned by create function

**Example:**
```lua
function sysCall_cleanup()
    simExtVelodyneROS_destroyVelodyneROSModel(velodyneHandle)
end
```

---

### Published Topics

| Topic | Message Type | Description | Frequency |
|-------|--------------|-------------|-----------|
| `/velodyne/points2` | `sensor_msgs/msg/PointCloud2` | 3D point cloud data | Per revolution |

#### PointCloud2 Message Structure

```yaml
header:
  stamp: Current ROS 2 time
  frame_id: "os1_sensor" or "velodyne"  # Configurable
height: 1                                # Unordered cloud
width: <number_of_points>
fields:
  - name: x
    offset: 0
    datatype: 7  # FLOAT32
    count: 1
  - name: y
    offset: 4
    datatype: 7  # FLOAT32
    count: 1
  - name: z
    offset: 8
    datatype: 7  # FLOAT32
    count: 1
  - name: intensity
    offset: 12
    datatype: 7  # FLOAT32
    count: 1
is_bigendian: false
point_step: 16      # 4 fields × 4 bytes
row_step: <width * point_step>
data: <binary point cloud data>
is_dense: false
```

**Coordinate Frame:** Global (odom frame) with automatic motion compensation

---

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

## 🔍 Troubleshooting

### Common Issues and Solutions

<details>
<summary><b>🔴 Plugin Doesn't Load</b></summary>

**Symptoms:**
- No plugin messages in CoppeliaSim console
- Plugin not listed in loaded plugins

**Solutions:**
1. Verify ROS 2 environment is sourced **before** starting CoppeliaSim
2. Check library exists: `ls -l $COPPELIASIM_ROOT_DIR/libv_repExtRosVelodyne.so`
3. Ensure correct library name (case-sensitive)
4. Check CoppeliaSim console for detailed error messages
5. Try starting CoppeliaSim from terminal to see error output

</details>

<details>
<summary><b>🔴 No Topics Published</b></summary>

**Symptoms:**
- `ros2 topic list` doesn't show `/velodyne/points2`
- No data appearing in RViz2

**Solutions:**
1. Check if node is running: `ros2 node list | grep velodyne`
2. Verify Velodyne model was created (check handle != -1 in Lua script)
3. Ensure simulation is **running** (not paused)
4. Confirm vision sensors are properly configured in scene
5. Check for Lua script errors in CoppeliaSim console

</details>

<details>
<summary><b>🔴 CoppeliaSim Crashes</b></summary>

**Symptoms:**
- Application crashes during simulation
- Segmentation fault errors

**⚠️ KNOWN ISSUE:** May conflict with simROS2 plugin

**Solutions:**
1. Reduce rotation frequency (try 5 Hz instead of 10 Hz)
2. Update to latest CoppeliaSim version (4.7.0+)
3. Check for memory leaks with `valgrind` (Linux)
4. Disable conflicting plugins
5. See [`/examples/troubleshooting.md`](./examples/troubleshooting.md) for detailed guidance

</details>

<details>
<summary><b>🔴 Empty or Invalid Point Clouds</b></summary>

**Symptoms:**
- Topic publishes but no points visible in RViz2
- Point cloud appears empty or corrupted

**Solutions:**
1. Check vision sensor settings (near/far clipping planes)
2. Verify objects are within sensor range
3. Check TF frame matches in RViz2 Fixed Frame setting
4. Ensure correct frame_id is set in code
5. Try different Color Transformer in RViz2 (Intensity, AxisColor, etc.)

</details>

<details>
<summary><b>🔴 Build Errors</b></summary>

**Symptoms:**
- Compilation fails with errors
- Missing dependencies

**Solutions:**
```bash
# Clean build completely
colcon build --packages-select coppeliasim_plugin_velodyne --cmake-clean-cache

# Reinstall dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Verify ROS 2 is sourced
echo $ROS_DISTRO  # Should show: humble, iron, jazzy, etc.
echo $AMENT_PREFIX_PATH  # Should not be empty

# Check compiler version
gcc --version  # Should be 7.0 or higher
```

</details>

> 📚 **For more detailed troubleshooting**, see [`/examples/troubleshooting.md`](./examples/troubleshooting.md)

---

## 📁 Project Structure

```
coppeliasim_plugin_velodyne/
│
├── 📄 README.md                           # This file - Main documentation
├── 📄 CMakeLists.txt                      # Build configuration
├── 📄 package.xml                         # ROS 2 package manifest
│
├── 📂 include/                            # C++ header files
│   └── coppeliasim_plugin_velodyne/
│       ├── ros_server_velodyne.h          # ROS 2 server interface
│       └── velodyneROSModel.h             # Velodyne model class
│
├── 📂 src/                                # C++ source files
│   ├── ros_server_velodyne.cpp            # ROS 2 server implementation
│   ├── velodyneROSModel.cpp               # Core Velodyne model logic
│   └── v_repExtVelodyneROS.cpp            # Plugin entry point
│
├── 📂 lua_scripts/                        # Lua scripts for CoppeliaSim
│   ├── README.md                          # Lua scripts documentation
│   ├── velodyne_main_ros2.lua             # Main controller script
│   ├── velodyne_sensor_script.lua         # Vision sensor processing
│   └── velodyne_ros2_lua.lua              # Alternative implementation
│
└── 📂 examples/                           # Usage examples and guides
    ├── README.md                          # Examples overview
    ├── sample_configuration.lua           # Configuration examples
    ├── alternative_approach.lua           # Simplified implementation
    └── troubleshooting.md                 # Detailed troubleshooting guide
```

---

## 📝 TODO

### Critical Priority
- [ ] 🔴 **Fix compatibility with simROS2 plugin** to prevent crashes
- [ ] 🔴 Verify ROS 2 topic publishing and message format correctness
- [ ] 🔴 Comprehensive testing with CoppeliaSim 4.7.x+

### High Priority
- [ ] 🟠 Add parameter server support for runtime configuration
- [ ] 🟠 Implement proper ROS 2 lifecycle node management
- [ ] 🟠 Performance optimization and memory leak detection
- [ ] 🟠 Support for additional Velodyne models (VLP-16, HDL-64E)

### Medium Priority
- [ ] 🟡 Add configurable QoS profiles
- [ ] 🟡 Implement comprehensive testing suite
- [ ] 🟡 Add CI/CD pipeline (GitHub Actions)
- [ ] 🟡 Create video tutorials and demos

### Low Priority
- [ ] 🟢 Add Docker support for easy deployment
- [ ] 🟢 Create ROS 2 launch files
- [ ] 🟢 Add configuration file support (YAML)
- [ ] 🟢 Improve documentation with diagrams

---

## ⚠️ Known Issues

| Issue | Severity | Status |
|-------|----------|--------|
| May crash CoppeliaSim when used with simROS2 plugin | 🔴 Critical | Under Investigation |
| Point cloud data format parsing errors | 🟠 High | Known |
| Revolution detection trigger unreliable | 🟠 High | Known |
| Performance degradation at high rotation frequencies | 🟡 Medium | Known |
| No parameter server integration | 🟡 Medium | Planned |
| Limited error handling and recovery | 🟢 Low | Improvement Needed |

> **Report new issues:** [GitHub Issues](https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2/issues)

---

## 🤝 Contributing

Contributions are welcome and appreciated! Whether you're fixing bugs, adding features, or improving documentation, your help makes this project better.

### How to Contribute

1. **Fork** the repository
2. **Create** a feature branch
   ```bash
   git checkout -b feature/amazing-improvement
   ```
3. **Make** your changes
4. **Test** thoroughly
5. **Commit** with clear messages
   ```bash
   git commit -m "Add: Brief description of changes"
   ```
6. **Push** to your fork
   ```bash
   git push origin feature/amazing-improvement
   ```
7. **Submit** a pull request

### Contribution Guidelines

When submitting a pull request, please include:

- ✅ **Clear description** of the problem/improvement
- ✅ **Test results** showing the changes work
- ✅ **Updated documentation** if behavior changes
- ✅ **Code follows** existing style and conventions
- ✅ **Commits are** meaningful and well-organized

### Areas Where Help is Needed

- 🐛 Bug fixes (especially simROS2 compatibility)
- 📚 Documentation improvements
- 🧪 Test coverage expansion
- 🎨 Code quality improvements
- 🌐 Multi-platform testing

---

## 📜 License

This project does not currently have a specified license.

> **Note:** If you plan to use this in production, please clarify the licensing with the repository owner.

---

## 🙏 Credits

### Original Work

This project is based on the original ROS 1 Velodyne plugin developed by ITVRoC:

**🔗 [ITVRoC/coppeliasim_plugin_velodyne](https://github.com/ITVRoC/coppeliasim_plugin_velodyne)**

The original repository provided the foundation for CoppeliaSim Velodyne sensor integration with ROS. This fork migrates that work to ROS 2, addressing compatibility with modern ROS distributions and CoppeliaSim versions.

### Contributors

| Role | Contributor | Contribution |
|------|-------------|--------------|
| **Original ROS 1 Plugin** | [ITVRoC](https://github.com/ITVRoC) | Initial implementation and core functionality |
| **ROS 2 Migration** | [ZiadMD](https://github.com/ZiadMD) | Migration to ROS 2, documentation, experimental scripts |
| **Simulation Platform** | [Coppelia Robotics](https://www.coppeliarobotics.com/) | CoppeliaSim/V-REP development |

### Acknowledgments

Special thanks to the **ITVRoC team** for their original work on the CoppeliaSim Velodyne plugin. Without their solid foundation, this ROS 2 migration would not have been possible. Their contribution to the robotics community is greatly appreciated.

---

## 📖 References

### Documentation
- 📘 [CoppeliaSim Documentation](https://www.coppeliarobotics.com/helpFiles/)
- 📗 [ROS 2 Documentation](https://docs.ros.org/)
- 📕 [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)
- 📙 [rclcpp API Reference](https://docs.ros2.org/latest/api/rclcpp/)

### Hardware & Sensors
- 🔧 [Velodyne LiDAR Official Site](https://velodynelidar.com/)
- 🔧 [LiDAR Point Cloud Processing](https://pcl.readthedocs.io/)

### Related Projects
- 🔗 [Original ITVRoC Plugin](https://github.com/ITVRoC/coppeliasim_plugin_velodyne)
- 🔗 [CoppeliaSim ROS 2 Interface](https://github.com/CoppeliaRobotics/simROS2)

---

## 💬 Support

Need help? Have questions? Here's where to find assistance:

### For This Project
- 🐛 **Bug Reports & Feature Requests**: [GitHub Issues](https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2/issues)
- 📚 **Examples & Setup Guides**: See the [`/examples`](./examples) folder
- 📖 **Troubleshooting**: Check [`/examples/troubleshooting.md`](./examples/troubleshooting.md)

### General ROS 2 & CoppeliaSim
- 💬 **CoppeliaSim Forum**: [forum.coppeliarobotics.com](https://forum.coppeliarobotics.com/)
- 💬 **ROS Answers**: [answers.ros.org](https://answers.ros.org/)
- 💬 **ROS Discourse**: [discourse.ros.org](https://discourse.ros.org/)

### Community
- 🌐 **ROS 2 Community**: [ros.org/community](https://www.ros.org/community/)
- 📺 **Tutorials**: Search "CoppeliaSim ROS 2" on YouTube

---

<div align="center">

### 🔗 Quick Links

**Repository**: [github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2](https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2)

**Original Project**: [github.com/ITVRoC/coppeliasim_plugin_velodyne](https://github.com/ITVRoC/coppeliasim_plugin_velodyne)

**Status**: 🔴 Experimental - Not Working

---

Made with ❤️ for the ROS 2 community

</div>
