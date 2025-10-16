# Lua Scripts for CoppeliaSim

⚠️ **WARNING: EXPERIMENTAL SCRIPTS**

This folder contains Lua scripts designed to be used inside CoppeliaSim for Velodyne sensor ROS2 integration.

## Files

### Core Scripts

- **`velodyne_main_ros2.lua`** - Main script to attach to the Velodyne base model
  - Handles ROS2 publishing
  - Manages rotation and revolution detection
  - Coordinates data from all vision sensors

- **`velodyne_sensor_script.lua`** - Customization script for vision sensors
  - Attach to each of the 4 vision sensors (sensor0-sensor3)
  - Processes depth data
  - Extracts Velodyne-formatted point cloud data

- **`velodyne_ros2_lua.lua`** - Alternative pure Lua implementation
  - Standalone approach with all logic in one script
  - Experimental version for testing

## Usage

These scripts are meant to be copied into CoppeliaSim scene objects, not executed directly.

### Quick Start

1. Open your CoppeliaSim scene with a Velodyne sensor model
2. Copy `velodyne_main_ros2.lua` into the Velodyne base object as a child script
3. Copy `velodyne_sensor_script.lua` into each of the 4 vision sensors as customization scripts
4. Configure parameters as needed
5. Start simulation

For detailed instructions, see the `/examples` folder.

## Important Notes

- These scripts are **experimental** and may cause crashes
- They are designed specifically for CoppeliaSim 4.7.0+
- Require the simROS2 plugin to be loaded
- May conflict with existing ROS2 integrations

## See Also

- `/examples` - Complete setup instructions and troubleshooting
- Main README.md - Project overview and installation
- ROS2_MIGRATION.md - Migration notes from ROS1

## Status

🔴 **Not Working** - These scripts are under development and do not currently function correctly.
