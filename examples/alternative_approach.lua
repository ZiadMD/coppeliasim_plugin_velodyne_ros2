--[[
    Alternative Approach: Simplified Velodyne ROS2 Script
    
    This is a simplified alternative implementation that may be more stable.
    Use this if the main script causes crashes or issues.
    
    KEY DIFFERENCES:
    - Single-threaded approach
    - Simplified data handling
    - Less aggressive memory management
    - Manual revolution tracking only
--]]

function sysCall_init()
    -- Basic ROS2 check
    simROS2 = require('simROS2')
    if not simROS2 then
        sim.addLog(sim.verbosity_scripterrors, "Cannot load simROS2")
        return
    end
    
    -- Get basic handles
    modelBase = sim.getObject('.')
    rotJoint = sim.getObject('./rotJoint')
    
    -- Simple configuration
    freq = 5  -- Lower frequency = more stable
    pi = math.pi
    
    -- Get sensors (simplified)
    sensors = {}
    for i = 0, 3 do
        local ok, h = pcall(sim.getObject, './sensor', {index=i})
        if ok then 
            sensors[i+1] = h
            sim.setExplicitHandling(h, 1)
        end
    end
    
    -- Simple publisher
    pub = simROS2.createPublisher('/velodyne_points', 'sensor_msgs/msg/PointCloud2')
    simROS2.publisherTreatUInt8ArrayAsString(pub)
    
    -- State
    packets = {}
    lastAngle = 0
    totalRotation = 0
    
    sim.addLog(sim.verbosity_scriptinfos, "Simplified Velodyne initialized")
end

function sysCall_actuation()
    if not rotJoint then return end
    
    -- Simple rotation
    local dt = sim.getSimulationTimeStep()
    local angle = sim.getJointPosition(rotJoint)
    local newAngle = angle + dt * freq * 2 * pi
    sim.setJointPosition(rotJoint, newAngle)
    
    -- Track rotation
    local delta = newAngle - lastAngle
    if delta < 0 then delta = delta + 2*pi end
    totalRotation = totalRotation + delta
    lastAngle = newAngle
end

function sysCall_sensing()
    if not simROS2 or #sensors == 0 then return end
    
    -- Simple sensor handling
    for _, s in ipairs(sensors) do
        sim.handleVisionSensor(s)
        simVision.sensorDepthMapToWorkImg(s)
        
        local trig, pkt = simVision.velodyneDataFromWorkImg(s, {500, 32}, 0.468)
        
        if pkt then
            table.insert(packets, pkt)
        end
    end
    
    -- Simple revolution check
    if totalRotation >= 2 * pi then
        if #packets > 0 then
            publishSimple()
            packets = {}
        end
        totalRotation = 0
    end
end

function publishSimple()
    -- Very simple publishing
    local data = table.concat(packets)
    local numPoints = math.floor(#data / 16)
    
    if numPoints == 0 then return end
    
    local msg = {
        header = {
            stamp = simROS2.getTime(),
            frame_id = 'velodyne'
        },
        height = 1,
        width = numPoints,
        fields = {
            {name='x', offset=0, datatype=7, count=1},
            {name='y', offset=4, datatype=7, count=1},
            {name='z', offset=8, datatype=7, count=1},
            {name='intensity', offset=12, datatype=7, count=1}
        },
        is_bigendian = false,
        point_step = 16,
        row_step = 16 * numPoints,
        data = string.sub(data, 1, numPoints * 16),
        is_dense = false
    }
    
    simROS2.publish(pub, msg)
    sim.addLog(sim.verbosity_scriptinfos, 
        string.format("Published %d points", numPoints))
end

function sysCall_cleanup()
    packets = {}
    sim.addLog(sim.verbosity_scriptinfos, "Cleanup done")
end


--[[
    USAGE NOTES:
    
    1. This script sacrifices some features for stability
    2. Uses lower point count (500 vs 1000)
    3. Lower default frequency (5 Hz vs 10 Hz)
    4. Simpler data concatenation
    5. No advanced error checking
    
    TRY THIS IF:
    - Main scripts cause crashes
    - Need basic functionality only
    - Want to debug issues
    - Testing ROS2 integration
    
    LIMITATIONS:
    - Fewer points per revolution
    - No intensity correction
    - Basic error handling
    - May miss some edge cases
--]]
