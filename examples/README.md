# CoppeliaSim Velodyne ROS2 Examples

This folder contains example scripts and snippets for using the Velodyne ROS2 plugin in CoppeliaSim.

⚠️ **WARNING**: These scripts are experimental and may not work correctly. Use at your own risk.

## Overview

The Velodyne sensor in CoppeliaSim consists of:
1. **Base Model** - The main Velodyne object with rotating joint
2. **4 Vision Sensors** - Individual sensors at different angles (HDL-32E configuration)
3. **Main Script** - Attached to the base model, handles ROS2 publishing
4. **Sensor Scripts** - Attached to each vision sensor, processes depth data

## Setup Instructions

### Step 1: Prepare Your CoppeliaSim Scene

1. Open CoppeliaSim
2. Load or create a scene with a Velodyne sensor model (e.g., VPL-16 or HDL-32E)
3. Ensure the model has:
   - A base object (main Velodyne body)
   - A rotating joint (rotJoint)
   - 4 vision sensors named: sensor0, sensor1, sensor2, sensor3

### Step 2: Add Scripts to Your Model

**Note:** All Lua scripts are located in the `/lua_scripts` folder.

#### For the Base Model:
1. Right-click on the Velodyne base object
2. Select "Add" → "Associated child script" → "Non-threaded"
3. Copy the contents of `/lua_scripts/velodyne_main_ros2.lua` into the script
4. Save the script

#### For Each Vision Sensor:
1. Right-click on each vision sensor (sensor0, sensor1, sensor2, sensor3)
2. Select "Add" → "Associated customization script"
3. Copy the contents of `/lua_scripts/velodyne_sensor_script.lua` into the script
4. Repeat for all 4 sensors

### Step 3: Configure Parameters

Edit the main script parameters as needed:
- `frequency`: Rotation speed in Hz (default: 10 Hz)
- `frame_id`: ROS2 frame name (default: 'velodyne')
- Topic name: Published to `/velodyne_points`

### Step 4: Run the Simulation

1. Ensure ROS2 is running and the simROS2 plugin is loaded
2. Start the simulation in CoppeliaSim
3. Check ROS2 topics: `ros2 topic list` should show `/velodyne_points`
4. Visualize in RViz2: `ros2 run rviz2 rviz2`

## Files in This Folder

- `README.md` - This file
- `sample_configuration.lua` - Sample configuration snippet
- `alternative_approach.lua` - Alternative implementation approach
- `troubleshooting.md` - Common issues and solutions

## Troubleshooting

If you encounter issues:
1. Check that simROS2 plugin is loaded (CoppeliaSim console)
2. Verify vision sensors are properly configured
3. Check ROS2 environment is sourced
4. See `troubleshooting.md` for more detailed solutions

## Known Issues

- May cause crashes with simROS2 plugin
- Point cloud data format may not be correctly parsed
- Revolution detection may not trigger properly
- Performance issues with high-frequency rotation

## Contributing

If you find solutions or improvements, please contribute to the main repository!
