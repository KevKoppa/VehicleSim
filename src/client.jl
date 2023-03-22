struct VehicleCommand
    steering_angle::Float64
    forward_force::Float64 # Rename to target_velocity or similar
    persist::Bool
    shutdown::Bool
end

function get_c()
    ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
    ret == 0 || error("unable to switch to raw mode")
    c = read(stdin, Char)
    ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    c
end

function keyboard_client(host::IPAddr=IPv4(0), port=4444; f_step = 1.0, s_step = π/10)
    socket = Sockets.connect(host, port)
    local state_msg
    @async while isopen(socket)
        state_msg = deserialize(socket)
    end
    
    forward_force = 0.0
    steering_angle = 0.0
    persist = true
    shutdown = false
    @info "Press 'q' at any time to terminate vehicle. Press 's' to shutdown simulator server."
    while persist && !shutdown && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            persist = false
        elseif key == 's'
            # shutdown server
            shutdown = true
        elseif key == 'i'
            # increase forward force
            forward_force += f_step
            @info "Target velocity: $forward_force"
        elseif key == 'k'
            # decrease forward force
            forward_force -= f_step
            @info "Target velocity: $forward_force"
        elseif key == 'j'
            # increase steering angle
            steering_angle += s_step
        elseif key == 'l'
            # decrease steering angle
            steering_angle -= s_step
        end
        cmd = VehicleCommand(steering_angle, forward_force, persist, shutdown)        
        Serialization.serialize(socket, cmd)
    end
end

function example_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()

    state_msg = VehicleState(zeros(13), zeros(12), false)
    @async while isopen(socket)
        state_msg = deserialize(socket)
    end
   
    shutdown = false
    persist = true
    while isopen(socket)
        position = state_msg.q[5:7]
        @info position
        if norm(position) >= 100
            shutdown = true
            persist = false
        end
        cmd = VehicleCommand(0.0, 2.5, persist, shutdown)
        serialize(socket, cmd) 
    end

end