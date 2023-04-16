"""
struct MyLocalizationType
    field1::Int
    field2::Float64
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end

function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    while true
        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end
        
        # process measurements

        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end
"""


# rng makes start = 32 children: 30, 28, 26, 24
function route(goal::Int, start::Int)

    # temporarily have map variable but change to function parameter 
    map::Dict{Int, RoadSegment} = training_map()

    function manhattan(goal::Int, start::Int) 
        p1 = map[goal].lane_boundaries[1].pt_b
        p2 = map[goal].lane_boundaries[2].pt_b
        p3 = map[start].lane_boundaries[1].pt_b
        p4 = map[start].lane_boundaries[2].pt_b
        
        pgoal = (p1 + p2)/2
        pstart = (p3 + p4)/2
    
        return sum(abs.(pgoal - pstart))
    end 

    function neighbors(road::Int)
        return map[road].children
    end

    function isgoal(goal::Int, start::Int)
        return goal == start
    end
    
    function hash(road::Int) 
        return road
    end

    result = astar(neighbors, start, goal; heuristic=manhattan, cost=manhattan, isgoal=isgoal, hashfn=hash, timeout=Inf, maxcost=Inf)
    if (result.status == :success)
        return result.path
    else
        return Vector{Int}[]
    end
end

# Q comes after P always
struct MidPath
    midP::SVector{2,Float64}
    midQ::SVector{2,Float64}
    speed_limit::Float64
    lane_types::Vector{LaneTypes}
end

function midpoints(paths::Vector{Int}) 
    # temporarily have map variable but change to function parameter 
    map::Dict{Int, RoadSegment} = training_map()
    n = length(paths)
    if n == 0
        return Vector{Path}[]
    end

    midpointPaths::Vector{MidPath} = []

    for i in 1:n
        lane = map[paths[i]]
        midP = (lane.lane_boundaries[1].pt_a + lane.lane_boundaries[2].pt_a)/2
        midQ = (lane.lane_boundaries[1].pt_b + lane.lane_boundaries[2].pt_b)/2
        
        push!(midpointPaths, MidPath(midP, midQ, lane.speed_limit, lane.lane_types)) 
        
        """
        differentiate between intersection and other lane types later 
        if lane.lane_types == intersection
            mid = (lane.lane_boundaries[1].pt_b + map[path[i]].lane_boundaries[2].pt_b)/2
        else
            mid = (lane.lane_boundaries[1].pt_b + map[path[i]].lane_boundaries[2].pt_b)/2
        end
        midpoints.push!(mid)
        """
    end

    return midpointPaths
end

# cross track error of vehicle point (distance between vehicle and path)
function CTE(ego::SVector{2,Float64}, path::MidPath) 
    midP = path.midP
    midQ = path.midQ
    v = [midQ[2] - midP[2]; -(midQ[1] - midP[1])]
    r = [midP[1] - ego[1]; midP[2] - ego[2]]
    return sum(v.*r)
end

# returns midPath that vehicle should travel should be incremented to next one
function iterateMidPath(ego::SVector{2,Float64}, path::MidPath)
    midP = path.midP
    
    dist = sqrt((ego - path.midP)'*((ego - path.midP)))
    #print("dist=$dist from ego=$ego to midP=$midP")
    if dist < 3
        
        return true
    else
        return false
    end
end

function path_planning(
    socket
    )
    #=
    localization_state_channel, 
    perception_state_channel, 
    map, 
    target_road_segment_id, 
    =#
    # do some setup
    #latest_localization_state = fetch(localization_state_channel)
    #latest_perception_state = fetch(perception_state_channel)
    # figure out what to do ... setup motion planning problem etc
    steering_angle = 0.0
    target_vel = 1.0
    cmd = VehicleCommand(steering_angle, target_vel, true)
    serialize(socket, cmd)
end

"""
function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)
        
        # process bounding boxes / run ekf / do what you think is good

        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        map, 
        target_road_segment_id, 
        socket)
    # do some setup
    while true
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        # figure out what to do ... setup motion planning problem etc
        steering_angle = 0.0
        target_vel = 0.0
        cmd = VehicleCommand(steering_angle, target_vel, true)
        serialize(socket, cmd)
    end
end

function isfull(ch:Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    @async while true
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        local measurement_msg
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
            else
                break
            end
        end
        target_map_segment = measurement_msg.target_segment
        ego_vehicle_id = measurement_msg.vehicle_id
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end

    @async localize(gps_channel, imu_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end
"""