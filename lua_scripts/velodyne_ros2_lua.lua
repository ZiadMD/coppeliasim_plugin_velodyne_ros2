--[[
    ⚠️ EXPERIMENTAL SCRIPT - USE WITH CAUTION ⚠️
    
    This is an experimental Lua script designed to be used inside CoppeliaSim.
    
    IMPORTANT NOTES:
    - This script is NOT fully functional and may cause crashes
    - Currently in development and testing phase
    - May conflict with the simROS2 plugin
    - Use at your own risk
    
    PURPOSE:
    This script should be attached to the Velodyne sensor model in CoppeliaSim
    to enable ROS2 PointCloud2 publishing functionality.
    
    USAGE:
    1. Open your CoppeliaSim scene
    2. Select the Velodyne model base object
    3. Add this script as a child script (non-threaded)
    4. Configure the parameters below as needed
    
    See the /examples folder for usage snippets and configuration examples.
--]]

-- Velodyne HDL-32E ROS2 Implementation in Pure Lua
-- This script uses CoppeliaSim's built-in Velodyne functions with ROS2 publishing
-- Main script attached to the Velodyne model base

function sysCall_init()
    -- Check if ROS2 is available
    if not simROS2 then
        simROS2 = require('simROS2')
    end
    
    if not simROS2 then
        sim.addLog(sim.verbosity_scripterrors, "simROS2 not available. Cannot initialize Velodyne.")
        return
    end
    
    -- Get the Velodyne model handle
    modelBase = sim.getObject('.')
    
    -- Get the rotating joint
    rotJoint = sim.getObject('./rotJoint')
    
    -- Get point cloud display handle (optional)
    local res, ptCloudHandle = pcall(sim.getObject, './ptCloud')
    if res then
        _ptCloudHandle = ptCloudHandle
    else
        _ptCloudHandle = -1
    end
    
    -- Get all 4 vision sensors (HDL-32E uses 4 sensors at different angles)
    visionSensors = {}
    for i = 0, 3 do
        local success, handle = pcall(sim.getObject, './sensor', {index=i})
        if success then
            table.insert(visionSensors, handle)
            -- Enable explicit handling for vision sensors
            sim.setExplicitHandling(handle, 1)
        else
            sim.addLog(sim.verbosity_scripterrors, string.format("Failed to get sensor %d", i))
        end
    end
    
    if #visionSensors == 0 then
        sim.addLog(sim.verbosity_scripterrors, "No vision sensors found!")
        return
    end
    
    sim.addLog(sim.verbosity_scriptinfos, string.format("Found %d vision sensors", #visionSensors))
    
    -- Velodyne parameters
    frequency = 10  -- 10 Hz rotation frequency (adjustable: 5-20 Hz)
    PI_VAL = math.pi
    
    -- Velodyne data collection
    allPackedPackets = {}
    revolutionComplete = false
    pointsPerRevolution = 1000  -- Points per revolution
    lasersPerFiring = 32  -- HDL-32E has 32 lasers
    verticalFOV = 0.467748  -- Vertical field of view in radians (~26.8 degrees)
    
    -- ROS2 PointCloud2 Publisher
    pointCloudPub = simROS2.createPublisher('/velodyne_points', 'sensor_msgs/msg/PointCloud2')
    simROS2.publisherTreatUInt8ArrayAsString(pointCloudPub)
    sim.addLog(sim.verbosity_scriptinfos, "ROS2 PointCloud2 publisher created on /velodyne_points")
    
    -- Initialize PointCloud2 message structure
    pointCloudBuffer = {
        header = {
            stamp = simROS2.getTime(),
            frame_id = 'velodyne'
        },
        height = 1,  -- unordered point cloud
        width = 0,
        fields = {
            {name = 'x', offset = 0, datatype = 7, count = 1},  -- FLOAT32 = 7
            {name = 'y', offset = 4, datatype = 7, count = 1},
            {name = 'z', offset = 8, datatype = 7, count = 1},
            {name = 'intensity', offset = 12, datatype = 7, count = 1}  -- Optional intensity
        },
        is_bigendian = false,
        point_step = 16,  -- 4 floats * 4 bytes
        row_step = 0,
        data = {},
        is_dense = false
    }
end

function sysCall_actuation()
    if not rotJoint then return end
    
    -- Rotate the Velodyne sensor
    local dt = sim.getSimulationTimeStep()
    local currentAngle = sim.getJointPosition(rotJoint)
    local angleIncrement = dt * frequency * 2 * PI_VAL
    sim.setJointPosition(rotJoint, currentAngle + angleIncrement)
end

function sysCall_sensing()
    if not simROS2 or not visionSensors or #visionSensors == 0 then 
        return 
    end
    
    -- Reset revolution flag
    revolutionComplete = false
    
    -- Process each of the 4 vision sensors
    for sensorIndex, sensorHandle in ipairs(visionSensors) do
        -- Handle the vision sensor explicitly
        sim.handleVisionSensor(sensorHandle)
        
        -- Convert sensor depth map to work image
        simVision.sensorDepthMapToWorkImg(sensorHandle)
        
        -- Extract Velodyne data from work image
        local trigger, packedPacket = simVision.velodyneDataFromWorkImg(
            sensorHandle, 
            {pointsPerRevolution, lasersPerFiring}, 
            verticalFOV
        )
        
        -- Check if we completed a revolution
        if trigger then
            revolutionComplete = true
        end
        
        -- Collect packed packets
        if packedPacket then
            table.insert(allPackedPackets, packedPacket)
        end
    end
    
    -- When a full revolution is complete, process and publish all packets
    if revolutionComplete and #allPackedPackets > 0 then
        processAndPublishPackets()
        allPackedPackets = {}  -- Clear for next revolution
    end
end

function processAndPublishPackets()
    -- Convert packed packets to PointCloud2 format
    local points = {}
    
    for _, packedPacket in ipairs(allPackedPackets) do
        -- Unpack the Velodyne packet data
        -- Each packet contains point data in packed format
        -- Format: typically x, y, z, intensity values
        
        if packedPacket and #packedPacket > 0 then
            -- The packed packet is typically a string or byte array
            -- We need to unpack it properly
            local numPoints = #packedPacket / 16  -- Assuming 4 floats per point
            
            for i = 0, numPoints - 1 do
                local offset = i * 16
                -- Extract x, y, z, intensity (this is simplified - actual unpacking may differ)
                if offset + 15 <= #packedPacket then
                    table.insert(points, {
                        data = string.sub(packedPacket, offset + 1, offset + 16)
                    })
                end
            end
        end
    end
    
    -- Build and publish PointCloud2 message
    if #points > 0 then
        publishPointCloudFromPackets(points)
    end
end

function publishPointCloudFromPackets(points)
    -- Update PointCloud2 message
    pointCloudBuffer.header.stamp = simROS2.getTime()
    pointCloudBuffer.width = #points
    pointCloudBuffer.row_step = pointCloudBuffer.point_step * pointCloudBuffer.width
    
    -- Concatenate all point data
    local dataString = ""
    for _, point in ipairs(points) do
        dataString = dataString .. point.data
    end
    
    pointCloudBuffer.data = dataString
    
    -- Publish to ROS2
    simROS2.publish(pointCloudPub, pointCloudBuffer)
    
    sim.addLog(sim.verbosity_scriptinfos, 
        string.format("Published PointCloud2 with %d points", #points))
end

function sysCall_cleanup()
    -- Clear packet buffer
    allPackedPackets = {}
    
    -- ROS2 handles publisher cleanup automatically
    sim.addLog(sim.verbosity_scriptinfos, "Velodyne ROS2 cleanup complete")
end

-- Note: Each of the 4 vision sensors should have their own customization script
-- with the following sysCall_vision function:
--[[
function sysCall_init()
end

function sysCall_vision(inData)
    local retVal={}
    retVal.trigger=false
    retVal.packedPackets={}
    simVision.sensorDepthMapToWorkImg(inData.handle)
    local trig,packedPacket=simVision.velodyneDataFromWorkImg(inData.handle,{1000,32},0.467748)
    if trig then retVal.trigger=true end
    if packedPacket then retVal.packedPackets[#retVal.packedPackets+1]=packedPacket end
    return retVal
end
--]]
