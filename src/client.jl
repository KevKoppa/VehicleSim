struct VehicleCommand
    steering_angle::Float64
    velocity::Float64
    controlled::Bool
end

function get_c()
    ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
    ret == 0 || error("unable to switch to raw mode")
    c = read(stdin, Char)
    ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    c
end

function keyboard_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)
    msg = deserialize(socket) # Visualization info
    println("msg: $msg") # prints out client followcam address 
    #println("hello")

    @async while isopen(socket)
        #sleep(0.001)
        state_msg = deserialize(socket)
        #println("msg: $state_msg")

        # MeasurementMessage - vehicle_id::Int, target_segment::Int, measurements::Vector{Measurement}
        measurements = state_msg.measurements 
        
        num_cam = 0
        num_imu = 0
        num_gps = 0
        num_gt = 0
        for meas in measurements
            if meas isa GroundTruthMeasurement
                num_gt += 1
            elseif meas isa CameraMeasurement
                num_cam += 1
            elseif meas isa IMUMeasurement
                num_imu += 1
            elseif meas isa GPSMeasurement
                num_gps += 1
            end
        end
  #      @info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
    end
    
    target_velocity = 0.0
    steering_angle = 0.0
    controlled = true
    @info "Press 'q' at any time to terminate vehicle."
    while controlled && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            controlled = false
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Keyboard Client."
        elseif key == 'i'
            # increase target velocity
            target_velocity += v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'k'
            # decrease forward force
            target_velocity -= v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'j'
            # increase steering angle
            steering_angle += s_step
            @info "Target steering angle: $steering_angle"
        elseif key == 'l'
            # decrease steering angle
            steering_angle -= s_step
            @info "Target steering angle: $steering_angle"
        end
        cmd = VehicleCommand(steering_angle, target_velocity, controlled)
        serialize(socket, cmd)
    end
end

function example_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()
    (; chevy_base) = load_mechanism()

    """
    @async while isopen(socket)
        state_msg = deserialize(socket)
    end
    """

    shutdown = false
    persist = true
    while isopen(socket)
        #position = state_msg.q[5:7]
        @info position
        if norm(position) >= 100
            shutdown = true
            persist = false
        end
        cmd = VehicleCommand(0.0, 2.5, persist, shutdown)
        serialize(socket, cmd) 
    end
end

function auto_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)

    msg = deserialize(socket) # Visualization info
    @info msg
    
    
    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    #localization_state_channel = Channel{MyLocalizationType}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)
    

    #target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    #ego_id = 1 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)
    
    @info "Press 'q' at any time to terminate vehicle."
    
    # for print statements
    i = 0
    j = 0

    # collect data from ground truth channel 
    @async while isopen(socket)
        sleep(0.001)
        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        !received && continue

        # MeasurementMessage - vehicle_id::Int, target_segment::Int, measurements::Vector{Measurement}
        measurements = measurement_msg.measurements 
        
        num_cam = 0
        num_imu = 0
        num_gps = 0
        num_gt = 0
        for meas in measurements
            if meas isa GroundTruthMeasurement
                num_gt += 1
                !isfull(gt_channel) && put!(gt_channel, meas)
            elseif meas isa CameraMeasurement
                num_cam += 1
            elseif meas isa IMUMeasurement
                num_imu += 1
            elseif meas isa GPSMeasurement
                num_gps += 1
            end
        end
        
        # print statements for debugging 
        #=
        if i < 50
            @info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
            i += 1
        end
        =#
    end
    
    
    routes::Vector{Int} = route(38,32) # sample route for car 
    midpoint_paths::Vector{MidPath} = midpoints(routes) # list of midpoint paths created 

    m = 2
    taup = 0.1
    taud = 0.3
    controlled = true
    target_velocity = 6 # velocity >=2.5 is out of control 
    steering_angle = 0.0
    error = 0
    init_error = 0
    first_iter = true
    cond = true # condition for main while loop to execute 

    # main while loop for execution 
    @async while cond
        sleep(0.001)
        meas = fetch(gt_channel)

        if j < 5
            print("in while loop")
            ego_id = meas.vehicle_id
            print("ego $ego_id\n")
            j += 1
        end

        # get most recent data from channel 
        while length(gt_channel.data)  > 1
            new_meas = take!(gt_channel)
            if  new_meas.time > meas.time  #&& meas.vehicle_id == ego_id
                meas = new_meas
            end
        end
        
        # New ego calculates the point 2/3 * length in front of center of car using quaternion measurements
        # Didn't change performance relative to using center of car before adopting circular turn error
        # Significantly improved turns after implementing curved turn error with dist 7
        # adding length failed, 1/2 length best
        w = meas.orientation[1]
        x = meas.orientation[2]
        y = meas.orientation[3]
        z = meas.orientation[4]

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan(t3, t4)
        ego = SA[meas.position[1] ; meas.position[2]] + 2*meas.size[1]/3*SA[cos(yaw_z), sin(yaw_z)]
        

        # old ego only considered center of car 
        #ego = SA[meas.position[1] ; meas.position[2]]

        # takes in ego position and next midpoint path and determines whether path followed should 
        # change to that next path if close enough 
        if (iterateMidPath(ego, midpoint_paths[m]))
            print("changed mid path from \n")
            oldMidP = midpoint_paths[m-1].midP
            oldMidQ = midpoint_paths[m-1].midQ
            newMidP = midpoint_paths[m].midP
            newMidQ = midpoint_paths[m].midQ
            averageR = 1/midpoint_paths[m].avg_curvature
            print("$oldMidP -$oldMidQ to $newMidP - $newMidQ $averageR\n")

            m += 1
        end

        len = length(midpoint_paths)
        if (m > len)
            cmd = VehicleCommand(0, 0, controlled)
            serialize(socket, cmd)
            cond = false
            continue
        end
        
        if (!first_iter)
            init_error = error
        end
        error = CTE(ego, midpoint_paths[m-1])

        if (first_iter)
            dev = 0
            init_error = error
        else
            dev = error-init_error
        end

        # TODO tinker with tau values to minimize overshoot 
        steering_angle = -taup*error - taud*dev

        # ensures that steering angle isn't huge value 
        # 0.5 steering allows for enough steering ability while not over steering
        # 0.3 not enough steering ability
        if (steering_angle > 0.5)
            steering_angle = 0.5
        elseif (steering_angle < -0.5)
            steering_angle = -0.5
        end
        
        first_iter = false
        cmd = VehicleCommand(steering_angle, target_velocity, controlled)
        serialize(socket, cmd)
    end
    
    # if 'q' clicked, then main thread terminates
    while controlled && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            controlled = false
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Keyboard Client."
        end
    end
end