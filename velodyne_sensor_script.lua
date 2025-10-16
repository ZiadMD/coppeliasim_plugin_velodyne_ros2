-- Vision Sensor Customization Script
-- Attach this script to EACH of the 4 vision sensors in the Velodyne model
-- This handles the depth data processing for each sensor

function sysCall_init()
    -- Initialization (if needed)
end

function sysCall_vision(inData)
    -- This callback is automatically called by CoppeliaSim for vision sensors
    local retVal = {}
    retVal.trigger = false
    retVal.packedPackets = {}
    
    -- Convert sensor depth map to work image
    simVision.sensorDepthMapToWorkImg(inData.handle)
    
    -- Extract Velodyne-formatted data from the work image
    -- Parameters:
    --   inData.handle: the vision sensor handle
    --   {1000, 32}: [points per revolution, number of lasers]
    --   0.467748: vertical field of view in radians (~26.8 degrees)
    local trig, packedPacket = simVision.velodyneDataFromWorkImg(
        inData.handle, 
        {1000, 32}, 
        0.467748
    )
    
    -- Set trigger flag if a full revolution is detected
    if trig then 
        retVal.trigger = true 
    end
    
    -- Add the packed packet to the return value
    if packedPacket then 
        retVal.packedPackets[#retVal.packedPackets + 1] = packedPacket 
    end
    
    return retVal
end
