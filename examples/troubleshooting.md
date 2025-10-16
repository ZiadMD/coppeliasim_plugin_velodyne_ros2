# Troubleshooting Guide

## Common Issues and Solutions

### Issue 1: simROS2 Plugin Not Found

**Symptoms:**
- Error message: "simROS2 not available"
- Script fails to initialize

**Solutions:**
1. Verify ROS2 is installed and sourced:
   ```bash
   source /opt/ros/humble/setup.bash  # or your ROS2 distro
   ```

2. Check if simROS2 plugin is in CoppeliaSim plugins folder:
   ```bash
   ls $COPPELIASIM_ROOT_DIR/libsimROS2.so
   ```

3. Start CoppeliaSim from a terminal with ROS2 sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   cd $COPPELIASIM_ROOT_DIR
   ./coppeliaSim.sh
   ```

4. Check CoppeliaSim console for plugin loading errors

---

### Issue 2: No Point Cloud Published

**Symptoms:**
- Simulation runs but no data on `/velodyne_points`
- `ros2 topic list` doesn't show the topic

**Solutions:**
1. Check if ROS2 publisher was created successfully:
   - Look for "ROS2 PointCloud2 publisher created" in CoppeliaSim console

2. Verify the topic with:
   ```bash
   ros2 topic list
   ros2 topic echo /velodyne_points
   ```

3. Check vision sensors are properly configured:
   - Each should have the sensor script attached
   - Verify sensor names: sensor0, sensor1, sensor2, sensor3

4. Ensure explicit handling is enabled for sensors

---

### Issue 3: CoppeliaSim Crashes

**Symptoms:**
- Simulation crashes when starting
- Crashes when Velodyne completes a revolution
- Random crashes during simulation

**Solutions:**
1. **KNOWN ISSUE**: This plugin may conflict with simROS2
   - Try reducing rotation frequency
   - Try publishing less frequently

2. Check memory usage:
   - Large point clouds can cause memory issues
   - Reduce POINTS_PER_REVOLUTION

3. Update CoppeliaSim to latest version (4.7.0+)

4. Check for conflicting scripts or plugins

---

### Issue 4: Vision Sensors Not Found

**Symptoms:**
- Error: "Failed to get sensor X"
- "No vision sensors found!"

**Solutions:**
1. Verify sensor hierarchy in CoppeliaSim scene tree:
   ```
   VelodyneModel
   ├── sensor0
   ├── sensor1
   ├── sensor2
   └── sensor3
   ```

2. Check sensor naming:
   - Must be named exactly: sensor, sensor#0, sensor#1, etc.
   - Or adjust the script to match your naming

3. Ensure sensors are children of the base model

4. Try using absolute paths instead of relative:
   ```lua
   local sensorPath = '/VelodyneVPL_16/sensor'
   handle = sim.getObject(sensorPath, {index=i})
   ```

---

### Issue 5: Invalid PointCloud2 Message

**Symptoms:**
- RViz2 shows errors when visualizing
- "Size mismatch" errors in console
- Point cloud appears corrupted

**Solutions:**
1. Verify point_step and data size match:
   - point_step should be 16 bytes (4 floats)
   - data length should be width × height × point_step

2. Check data is properly packed:
   - Each point: x, y, z, intensity (4 bytes each)
   - No padding or extra bytes

3. Ensure is_bigendian matches system architecture

4. Try different PointCloud2 field configurations

---

### Issue 6: Revolution Detection Doesn't Work

**Symptoms:**
- Revolution never completes
- No data published even though simulation is running
- Trigger flag never set

**Solutions:**
1. **KNOWN ISSUE**: Automatic trigger may not work reliably

2. Use manual revolution tracking:
   ```lua
   -- Track angle manually
   if accumulatedAngle >= 2 * math.pi then
       publishPointCloud()
       accumulatedAngle = 0
   end
   ```

3. Check rotation joint is actually rotating:
   ```lua
   sim.addLog(sim.verbosity_scriptinfos, 
       string.format("Current angle: %.2f", currentAngle))
   ```

4. Verify frequency and time step are reasonable

---

### Issue 7: Empty or Very Few Points

**Symptoms:**
- Point cloud published but contains few/no points
- "No complete points to publish" warning

**Solutions:**
1. Check vision sensor settings:
   - Near/far clipping planes
   - Resolution
   - Depth calculation enabled

2. Verify objects are in sensor range

3. Increase POINTS_PER_REVOLUTION

4. Check sensor angles and FOV

---

## Debugging Tips

### Enable Verbose Logging

Add more logging to track execution:

```lua
sim.addLog(sim.verbosity_scriptinfos, "Checkpoint 1")
sim.addLog(sim.verbosity_scriptinfos, 
    string.format("Value: %d", someVariable))
```

### Check ROS2 Topics

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /velodyne_points

# Echo messages (may be large!)
ros2 topic echo /velodyne_points --max-count 1

# Check publish rate
ros2 topic hz /velodyne_points
```

### Visualize in RViz2

```bash
ros2 run rviz2 rviz2
```

Then:
1. Set Fixed Frame to "velodyne" or "odom"
2. Add PointCloud2 display
3. Set topic to /velodyne_points
4. Adjust point size and color

### Check TF Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo odom velodyne
```

---

## Still Having Issues?

1. Check the main project README for updates
2. Review the ROS2_MIGRATION.md document
3. Check GitHub issues: https://github.com/ZiadMD/coppeliasim_plugin_velodyne_ros2/issues
4. Remember: This is experimental software!

---

## Reporting Bugs

When reporting issues, please include:
- CoppeliaSim version
- ROS2 distro and version
- Error messages from CoppeliaSim console
- Relevant script sections
- Steps to reproduce
