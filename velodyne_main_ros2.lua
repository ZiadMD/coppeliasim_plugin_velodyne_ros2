--[[
    ⚠️ EXPERIMENTAL SCRIPT - USE WITH CAUTION ⚠️
    
    This is an experimental Lua script designed to be used inside CoppeliaSim.
    
    IMPORTANT NOTES:
    - This script is NOT fully functional and may cause crashes
    - Currently in development and testing phase
    - May conflict with the simROS2 plugin
    - Use at your own risk
    
    PURPOSE:
    Main script for Velodyne HDL-32E ROS2 integration.
    Attach this to the Velodyne base model in CoppeliaSim.
    
    USAGE:
    1. Open your CoppeliaSim scene with a Velodyne model
    2. Select the Velodyne base object
    3. Add this script as a child script (non-threaded)
    4. Ensure the 4 vision sensors have the sensor script attached
    
    See the /examples folder for complete setup instructions.
--]]

-- Velodyne HDL-32E ROS2 Main Script
-- Attach this to the Velodyne base model
-- The 4 vision sensors handle their own data collection via sysCall_vision

function sysCall_init()
    -- Check if ROS2 is available
    if not simROS2 then
        local success, result = pcall(require, 'simROS2')
        if success then
            simROS2 = result
        else
            sim.addLog(sim.verbosity_scripterrors, "simROS2 not available")
            return
        end
    end
    
    -- Get handles
    modelBase = sim.getObject('.')
    rotJoint = sim.getObject('./rotJoint')
    
    -- Velodyne parameters
    frequency = 10  -- Hz, rotation frequency
    PI_VAL = math.pi
    
    -- Get all 4 vision sensors
    visionSensors = {}
    for i = 0, 3 do
        local success, handle = pcall(sim.getObject, './sensor', {index=i})
        if success then
            table.insert(visionSensors, handle)
            -- Enable explicit handling
            sim.setExplicitHandling(handle, 1)
        end
    end
    
    sim.addLog(sim.verbosity_scriptinfos, string.format("Found %d vision sensors", #visionSensors))
    
    -- ROS2 Publisher
    pointCloudPub = simROS2.createPublisher('/velodyne_points', 'sensor_msgs/msg/PointCloud2')
    simROS2.publisherTreatUInt8ArrayAsString(pointCloudPub)
    
    -- Point cloud accumulator
    accumulatedPackets = {}
    revolutionTriggered = false
    
    -- Manual revolution tracking (since trigger doesn't work)
    lastAngle = 0
    accumulatedAngle = 0
    revolutionCounter = 0
    
    sim.addLog(sim.verbosity_scriptinfos, "Velodyne ROS2 initialized")
end

function sysCall_actuation()
    if not rotJoint then return end
    
    -- Rotate the sensor
    local dt = sim.getSimulationTimeStep()
    local currentAngle = sim.getJointPosition(rotJoint)
    local newAngle = currentAngle + dt * frequency * 2 * PI_VAL
    sim.setJointPosition(rotJoint, newAngle)
    
    -- Track rotation for manual revolution detection
    local angleDelta = newAngle - lastAngle
    if angleDelta < -PI_VAL then
        angleDelta = angleDelta + 2 * PI_VAL
    elseif angleDelta > PI_VAL then
        angleDelta = angleDelta - 2 * PI_VAL
    end
    
    accumulatedAngle = accumulatedAngle + math.abs(angleDelta)
    lastAngle = newAngle
end

function sysCall_sensing()
    if not simROS2 or #visionSensors == 0 then return end
    
    local currentPackets = {}
    
    -- Process each vision sensor
    for _, sensorHandle in ipairs(visionSensors) do
        -- Explicitly handle the sensor to trigger depth capture
        sim.handleVisionSensor(sensorHandle)
        
        -- Convert depth map to work image
        simVision.sensorDepthMapToWorkImg(sensorHandle)
        
        -- Extract Velodyne data
        -- Parameters: {points_per_revolution, lasers_per_firing}, vertical_FOV_radians
        local trigger, packedPacket = simVision.velodyneDataFromWorkImg(
            sensorHandle, 
            {1000, 32},  -- 1000 points per revolution, 32 laser beams
            0.467748     -- ~26.8 degrees vertical FOV
        )
        
        -- Collect the packed packet data
        if packedPacket then
            table.insert(currentPackets, packedPacket)
        end
    end
    
    -- Accumulate packets
    for _, packet in ipairs(currentPackets) do
        table.insert(accumulatedPackets, packet)
    end
    
    -- Manual revolution detection: Check if we've rotated 360 degrees
    if accumulatedAngle >= 2 * PI_VAL then
        if #accumulatedPackets > 0 then
            revolutionCounter = revolutionCounter + 1
            sim.addLog(sim.verbosity_scriptinfos, 
                string.format("Revolution #%d complete (%.2f degrees) with %d packets", 
                    revolutionCounter, math.deg(accumulatedAngle), #accumulatedPackets))
            publishPointCloud()
            accumulatedPackets = {}
        end
        -- Reset accumulated angle for next revolution
        accumulatedAngle = accumulatedAngle - 2 * PI_VAL
    end
end

function publishPointCloud()
    -- Combine all packets into a single data buffer
    local combinedData = ""
    local totalBytes = 0
    
    for _, packet in ipairs(accumulatedPackets) do
        if type(packet) == "string" then
            combinedData = combinedData .. packet
            totalBytes = totalBytes + #packet
        end
    end
    
    if totalBytes == 0 then
        sim.addLog(sim.verbosity_scriptwarnings, "No data to publish")
        return
    end
    
    -- Calculate actual number of complete points (must be exact)
    local pointStep = 16  -- 4 floats × 4 bytes each
    local totalPoints = math.floor(totalBytes / pointStep)
    local validDataSize = totalPoints * pointStep
    
    -- Trim data to exact size (remove incomplete points at the end)
    if totalBytes > validDataSize then
        combinedData = string.sub(combinedData, 1, validDataSize)
        sim.addLog(sim.verbosity_scriptwarnings, 
            string.format("Trimmed %d bytes from point cloud data (incomplete point)", 
                totalBytes - validDataSize))
    end
    
    if totalPoints == 0 then
        sim.addLog(sim.verbosity_scriptwarnings, "No complete points to publish")
        return
    end
    
    -- Create PointCloud2 message with exact dimensions
    local msg = {
        header = {
            stamp = simROS2.getTime(),
            frame_id = 'velodyne'  -- Matches TF frame published by Cuboid script
        },
        height = 1,
        width = totalPoints,
        fields = {
            {name = 'x', offset = 0, datatype = 7, count = 1},  -- FLOAT32
            {name = 'y', offset = 4, datatype = 7, count = 1},
            {name = 'z', offset = 8, datatype = 7, count = 1},
            {name = 'intensity', offset = 12, datatype = 7, count = 1}
        },
        is_bigendian = false,
        point_step = pointStep,
        row_step = pointStep * totalPoints,
        data = combinedData,
        is_dense = false
    }
    
    -- Verify message consistency before publishing
    local expectedSize = msg.width * msg.height * msg.point_step
    local actualSize = #msg.data
    
    if expectedSize ~= actualSize then
        sim.addLog(sim.verbosity_scripterrors, 
            string.format("Size mismatch: expected %d, got %d bytes. Not publishing.", 
                expectedSize, actualSize))
        return
    end
    
    -- Publish
    simROS2.publish(pointCloudPub, msg)
    
    sim.addLog(sim.verbosity_scriptinfos, 
        string.format("Published PointCloud2: %d points (%d bytes)", totalPoints, actualSize))
end

function sysCall_cleanup()
    accumulatedPackets = {}
    sim.addLog(sim.verbosity_scriptinfos, "Velodyne ROS2 cleanup complete")
end
