--[[
    Sample Configuration Snippet for CoppeliaSim Velodyne Model
    
    This snippet shows various configuration options you can use
    in your Velodyne scripts.
--]]

-- ============================================
-- BASIC CONFIGURATION
-- ============================================

-- Rotation frequency (Hz)
-- Common values: 5, 10, 15, 20
-- Lower = more points per revolution, slower rotation
-- Higher = fewer points per revolution, faster rotation
local ROTATION_FREQUENCY = 10

-- Point cloud topic name
local POINTCLOUD_TOPIC = "/velodyne_points"

-- Frame ID for TF tree
local FRAME_ID = "velodyne"

-- Points per revolution
-- Velodyne VPL-16: typically 1800-3600
-- Velodyne HDL-32E: typically 1000-2000
local POINTS_PER_REVOLUTION = 1000

-- Number of laser beams
-- VPL-16: 16 lasers
-- HDL-32E: 32 lasers
local LASERS_PER_FIRING = 32

-- Vertical field of view (radians)
-- VPL-16: ~0.524 radians (30 degrees)
-- HDL-32E: ~0.468 radians (26.8 degrees)
local VERTICAL_FOV = 0.467748


-- ============================================
-- ADVANCED CONFIGURATION
-- ============================================

-- Enable/disable point cloud visualization in CoppeliaSim
local DISPLAY_POINTS = true

-- Point size for visualization
local POINT_SIZE = 2

-- Color coding based on distance
-- {near_distance, far_distance} in meters
local COLOR_DISTANCE_RANGE = {1, 4}

-- Scaling factor for displayed points
-- Slightly less than 1.0 prevents z-fighting with objects
local DISPLAY_SCALING = 0.999

-- Enable emissive rendering for points
-- Makes points glow, easier to see
local EMISSIVE_POINTS = true


-- ============================================
-- ROS2 CONFIGURATION
-- ============================================

-- QoS Profile settings (if needed for custom publisher)
local QOS_PROFILE = {
    reliability = "reliable",  -- or "best_effort"
    durability = "volatile",   -- or "transient_local"
    history = "keep_last",
    depth = 10
}

-- Message encoding
-- PointCloud2 data field encoding
local IS_BIGENDIAN = false
local POINT_STEP = 16  -- bytes per point (4 floats × 4 bytes)


-- ============================================
-- SENSOR HIERARCHY (for reference)
-- ============================================

--[[
    Velodyne Model Structure:
    
    VelodyneVPL_16 (or HDL-32E)
    ├── rotJoint (rotating joint)
    ├── sensor0 (vision sensor, 0° offset)
    ├── sensor1 (vision sensor, 90° offset)
    ├── sensor2 (vision sensor, 180° offset)
    ├── sensor3 (vision sensor, 270° offset)
    └── ptCloud (optional, for visualization)
    
    Main script: attached to base model
    Sensor scripts: attached to each vision sensor
--]]


-- ============================================
-- USAGE EXAMPLE
-- ============================================

--[[
    In your sysCall_init():
    
    -- Get vision sensors with error handling
    visionSensors = {}
    for i = 0, 3 do
        local success, handle = pcall(sim.getObject, './sensor', {index=i})
        if success then
            table.insert(visionSensors, handle)
            sim.setExplicitHandling(handle, 1)
        else
            sim.addLog(sim.verbosity_scripterrors, 
                string.format("Failed to get sensor %d", i))
        end
    end
    
    -- Create ROS2 publisher
    if simROS2 then
        pointCloudPub = simROS2.createPublisher(
            POINTCLOUD_TOPIC, 
            'sensor_msgs/msg/PointCloud2'
        )
        simROS2.publisherTreatUInt8ArrayAsString(pointCloudPub)
    end
--]]
