# coppeliasim_plugin_velodyne

This CoppeliaSim plugin publishes a full revolution of points in PointCloud2 format to ROS 2 under the topic `/velodyne_points`.
Instead of serializing the Velodyne data directly with Non-threaded scripts, this plugin uses C++ to greatly increase publication speed.

The points are global (relative to the odom frame). This plugin is meant to be used with sensors like the built-in model velodyne VPL-16.

The reason why the point cloud is published globally is because at every simulation step the points are converted to the origin frame in order to correct the mismatch due to the movement of the vehicle. You can modify the plugin or simply create a node that reads this point cloud and converts it to the base_link (or whatever the velodyne frame is) frame.

**Note:** This project has been migrated to ROS 2 and uses colcon build system. Compatible with CoppeliaSim 4.7.0+.

## Installation

After cloning this repository into your ROS 2 workspace, build the project with colcon:

```sh
cd ~/ros2_ws  # or your workspace directory
colcon build --packages-select coppeliasim_plugin_velodyne
source install/setup.zsh
```

The plugin library needs to be copied into the CoppeliaSim folder:

```sh
$ cp ~/ros2_ws/install/coppeliasim_plugin_velodyne/lib/libsimExtVelodyneROS2.so $COPPELIASIM_ROOT_DIR
```

## Node configuration 

- Edit `src/ros_server_velodyne.cpp` to define the published topic name. Default: `/velodyne/points2`. 
- Edit `src/velodyneROSModel.cpp` to define the frame ID of the published pointcloud. Default: `os1_sensor`.


## CoppeliaSim sensor configuration

This is a sample Lua script configuration for the stock Velodyne VPL16 sensor.
For ROS 2 integration, see the `velodyne_ros2_lua.lua` and related Lua scripts in the package.

```
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
    local displayScaling=0.999 -- so that points do not appear to disappear in objects


	-- added for local frame version
	h_velodyne_sensor = sim.getObjectHandle('velodyneVPL_16')
	

    _h=simExtVelodyneROS_createVelodyneROSModel(visionSensorHandles,frequency,options,pointSize,coloring_closeAndFarDistance,displayScaling, h_velodyne_sensor)
end

if (sim_call_type==sim.syscb_sensing) then
    --Boolean, true if data is published (via VelodyneROS plugin)
    local fullRev=simExtVelodyneROS_handleVelodyneROSModel(_h, sim.getSimulationTimeStep()) 
    --local fullRev=simExtVelodyneROS_handleVelodyneROSModel(_h, sim.getSimulationTimeStep()) --use this if the full 360 cloud is needed instead of 4 separated pointclouds
end

if (sim_call_type==sim.syscb_cleanup) then
    simExtVelodyneROS_destroyVelodyneROSModel(_h)
end
```

## TODO

- [ ] Configure the project to work properly within CoppeliaSim without crashing the simROS2 plugin
- [ ] Test compatibility with latest CoppeliaSim versions
- [ ] Verify ROS 2 topic publishing and message format
- [ ] Update plugin loading mechanism for ROS 2 integration
- [ ] Document any additional dependencies or configuration steps required
