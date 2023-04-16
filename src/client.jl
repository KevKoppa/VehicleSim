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

function isFull(ch::Channel)
    return length(ch.data) >= ch.sz_max
end

function auto_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)

    msg = deserialize(socket) # Visualization info
    @info msg

    map_segments = training_map()
    
    
    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    #localization_state_channel = Channel{MyLocalizationType}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)
    

    #target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    #ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)
    
    @info "Press 'q' at any time to terminate vehicle."
    
    i = 0
    j = 0
    #=
    @async while isopen(socket)
        if j < 100
            @info "in while loop"
            j += 1
        end
        sleep(0.001)
        measurement_msg = deserialize(socket)
        #println("msg: $state_msg")

        # MeasurementMessage - vehicle_id::Int, target_segment::Int, measurements::Vector{Measurement}

        target_map_segment = meas.target_segment
        ego_vehicle_id = meas.vehicle_id
        len = length(measurement_msg.measurements)
        @info "length $len"
        for meas in measurement_msg.measurements
            if meas isa GroundTruthMeasurement
                full = !isfull(gt_channel)
                if i < 10
                    @info "GroundTruthMeasurement: $meas $full"
                    i += 1
                end
                
                !isfull(gt_channel) && put!(gt_channel, meas)
            else 
                if j < 100
                    @info "ground truth not found"
                    j += 1
                end
            end
        end
        #@info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
    end
    =#

    @async while isopen(socket)
        sleep(0.001)
        #state_msg = deserialize(socket)
        local state_msg
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                state_msg = deserialize(socket)
            else
                break
            end
        end
        #println("msg: $ state_msg")

        # MeasurementMessage - vehicle_id::Int, target_segment::Int, measurements::Vector{Measurement}
        measurements = state_msg.measurements 
        
        num_cam = 0
        num_imu = 0
        num_gps = 0
        num_gt = 0
        for meas in measurements
            if meas isa GroundTruthMeasurement
                num_gt += 1
                !isFull(gt_channel) && put!(gt_channel, meas)
            elseif meas isa CameraMeasurement
                num_cam += 1
            elseif meas isa IMUMeasurement
                num_imu += 1
            elseif meas isa GPSMeasurement
                num_gps += 1
            end
        end
        #=
        if j < 20
            @info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
            j += 1
        end
        =#
    end
    
    routes::Vector{Int} = route(38,32)
    midpoint_paths::Vector{MidPath} = midpoints(routes)
    
    len1 = length(routes)
    len2 = length(midpoint_paths)
    #@info "routes - $len1 , midpoint_paths - $len2"
    #=
    for i in 1:len1
        route = map_segments[routes[i]]
        print("A1 ")
        print(route.lane_boundaries[1].pt_a)
        print("A2")
        print(route.lane_boundaries[2].pt_a)
        print("Mid ")
        print(midpoint_paths[i].midP)
        print("\n")
    end
    =#
    m = 2
    taup = 0.1
    taud = 0.2
    controlled = true
    target_velocity = 2.5
    steering_angle = 0.0
    error = 0
    init_error = 0
    first_iter = true
    while controlled && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            controlled = false
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Keyboard Client."
        end
        
        #=
        if i < 100
            print("$steering_angle, $target_velocity, $controlled")
            i += 1
        end
        =#
        
        meas = fetch(gt_channel)
        #len = length(gt_channel.data) 
        #=
        if j < 100
            @info "len: $len"
            j += 1
        end
        =#

        while length(gt_channel.data)  > 1
            #=
            if j < 100
                print("in while loop length - $len")
                    j += 1
            end
            =#
            new_meas = take!(gt_channel)
            if  new_meas.time > meas.time
                meas = new_meas
            end
        end

        if j < 200
            @info "meas: $meas"
            j += 1
        end
        
        #=
        w = meas.orientation[1]
        x = meas.orientation[2]
        y = meas.orientation[3]
        z = meas.orientation[4]

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan(t3, t4)
        ego = SA[meas.position[1] ; meas.position[2]] + meas.size[1]/4*SA[cos(yaw_z), sin(yaw_z)]
        =# 

        ego = SA[meas.position[1] ; meas.position[2]]
        #print("ego $ego")
        #println("calling iterateMidPath")
        if (iterateMidPath(ego, midpoint_paths[m]))
            m += 1
            print("changed mid path from ")
            midP = midpoint_paths[m].midP
            midQ = midpoint_paths[m].midQ

            print("$midP to $midQ\n")
        end

        #println("made it out of iterateMidPath")
        
        if (!first_iter)
            init_error = error
        end
        error = CTE(ego, midpoint_paths[m-1])
        #=
        if i < 700
            @info "error $error "
            print("midpoint ")
            print(midpoint_paths[m-1])
            print(" ego $ego")
            i += 1
        end
        =#
        if (first_iter)
            dev = 0
            init_error = error
        else
            dev = error-init_error
        end

        #print("dev $dev")
        steering_angle = -taup*error - taud*dev
        if (steering_angle > 0.5)
            steering_angle = 0.3
        elseif (steering_angle < -0.5)
            steering_angle = -0.3
        end

        #print(" steering_angle $steering_angle")
        #=
        if i < 3
            #print("here")
            @info "ego: $ego"
            i += 1
        end
        =#
        
        first_iter = false
        cmd = VehicleCommand(steering_angle, target_velocity, controlled)
        serialize(socket, cmd)
    end 
end