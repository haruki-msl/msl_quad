#!/usr/bin/env julia

########################################
## File Name: mbc_generate_code_from_sender_trajectory.jl
## Author: Haruki Nishimura (hnishimura@stanford.edu)
## Date Created: 2019/03/10
## Description: Create active motion-based communication Trajectory Code from sender's trajectory
########################################

using ActiveMBC
using JLD
using Plots
using RobotOS

pyplot();

@rosimport geometry_msgs.msg: Pose, PoseStamped
rostypegen()
import .geometry_msgs.msg: Pose, PoseStamped

mutable struct ROSWrapperCodeGen
    quad_ns_sender::String                              # ROS Namespace for sender
    true_class::Int64                                   # True trajectory class

    sender_command_sub::Subscriber{Pose}                # ROS sender command subscriber
    sender_pose_sub::Subscriber{PoseStamped}            # ROS sender pose subscriber

    initial_time::Time                                  # Initial ROS-time when the communiation process started
    schedule::Vector{Tuple{Int64,Time}}                 # Vector to store {trajectory_lap, measurement_time} schedule
    num_repeat_max::Int64                               # Maximum number of times sender's trajectory will be repeated (including prior initialization)

    simulation_flag::Bool                               # Flag for ROS-Gazebo simulation using StanfordMSL/mslquad
    schedule_ready_flag::Bool                           # Flag for this.schedule availability
    pose_ready_flag::Bool                               # Flag for sender pose message availability
    started_flag::Bool                                  # Flag for starting communication process
    xz_plane_flag::Bool                                 # If this is true, y's in Code are always constant.

    state_histories::Vector{Vector{Vector{Float64}}}    # Sender's state histories for generating Code
    Code::Vector{Vector{Float64}}                       # Generated Code of true_class
    sender_offset::Vector{Float64}                      # Gazebo simulation is quite inaccurate in position hold. This is a hack to correct it.
    duration::Float64                                   # Duration of sender's trajectory
    interval::Float64                                   # Interval time between two trajectory executions of sender
    dt::Float64                                         # Interval time between receiver's successive actions

    function ROSWrapperCodeGen(quad_ns_sender::String,
                               true_class::Int64,
                               xz_plane_flag::Bool,
                               num_repeat_max::Int64)
        this = new();
        this.quad_ns_sender = quad_ns_sender;
        this.true_class = true_class;
        this.schedule = Vector{Tuple{Int64,Time}}();
        this.state_histories = Vector{Vector{Vector{Float64}}}(undef, num_repeat_max);
        this.Code = Vector{Vector{Float64}}();
        this.duration = duration;
        this.interval = interval;
        this.dt = dt;
        this.num_repeat_max = num_repeat_max;
        this.xz_plane_flag = xz_plane_flag;
        this.sender_pose_sub = Subscriber("/"*quad_ns_sender*"/mavros/local_position/pose",
                                          PoseStamped,
                                          sender_pose_cb!,
                                          (this, ),
                                          queue_size = 10);
        this.sender_command_sub = Subscriber("/"*quad_ns_sender*"/command/pose",
                                             Pose,
                                             sender_command_cb!,
                                             (this, ),
                                             queue_size = 10);
        this.simulation_flag = begin
            has_param("/simulation") && get_param("/simulation")
        end;
        if this.simulation_flag
            println("Simulation mode");
        else
            println("Experiment mode");
        end
        this.pose_ready_flag = false;
        this.schedule_ready_flag = false;
        this.started_flag = false;
        return this
    end
end

@inline function sender_pose_cb!(sender_pose::PoseStamped, wrapper::ROSWrapperCodeGen)
    logdebug("sender pose message received.")
    current_time = sender_pose.header.stamp;
    if !(wrapper.pose_ready_flag)
        loginfo("First pose message received. Code generation ready...")
        wrapper.pose_ready_flag = true;
    end
    if wrapper.started_flag
        if !(wrapper.schedule_ready_flag)
            wrapper.initial_time = current_time;
            println("Starting code generation process at $(wrapper.initial_time)");
            wrapper.schedule = begin
                get_measurement_schedule_codegen(wrapper.initial_time,
                                                 wrapper.num_repeat_max)
            end;
            wrapper.schedule_ready_flag = true;
        end
        current_lap, next_measurement_time = wrapper.schedule[1];
        valid_measurement_time = begin
            current_time >= next_measurement_time - Duration(0.05) &&
            current_time <= next_measurement_time + Duration(0.05);
        end
        if valid_measurement_time
            println("Sender pose received at $(current_time) while next_measurement_time is $(next_measurement_time)");
            if wrapper.xz_plane_flag
                point = [sender_pose.pose.position.x,
                         1.8,
                         sender_pose.pose.position.z];
            else
                point = [sender_pose.pose.position.x,
                         sender_pose.pose.position.y,
                         sender_pose.pose.position.z];
            end
            try
                push!(wrapper.state_histories[current_lap], point - wrapper.sender_offset); # hack for correcting z in Gazebo
            catch
                println("New lap (lap $(current_lap)) initiated.")
                sender_p_in_code = Codebook_sender[wrapper.true_class][1][2:4];
                sender_p_actual = point;
                wrapper.sender_offset = sender_p_actual - sender_p_in_code;
                wrapper.state_histories[current_lap] = [point - wrapper.sender_offset]; # hack for correcting z in Gazebo
            end
            println("New point pushed.")
            if length(wrapper.schedule) == 1
                println("All laps completed.")
                # All the laps have completed. Generate Code.
                wrapper.Code =  sum(wrapper.state_histories)./current_lap;
                wrapper.started_flag = false;
                filename = joinpath(dirname(pathof(ActiveMBC)), "../resource/code_$(wrapper.true_class).jld");
                save(filename, "Code_$(wrapper.true_class)", wrapper.Code);
                println("Code saved at "*filename);
                if wrapper.xz_plane_flag
                    scatter([p[1] for p in wrapper.Code],
                            [p[3] for p in wrapper.Code],
                            aspect_ratio=1.0)
                    xaxis!("x")
                    yaxis!("z")
                    title!("Code $(wrapper.true_class)");
                    filename_png = joinpath(dirname(pathof(ActiveMBC)), "../resource/code_$(wrapper.true_class).png");
                    savefig(filename_png);
                    println("Code picture saved at "*filename_png);
                else
                    scatter3d([p[1] for p in wrapper.Code],
                              [p[2] for p in wrapper.Code],
                              [p[3] for p in wrapper.Code],
                              aspect_ratio=1.0)
                    xaxis!("x")
                    yaxis!("y")
                    title!("Code $(wrapper.true_class)");
                    filename_png = joinpath(dirname(pathof(ActiveMBC)), "../resource/code_$(wrapper.true_class).png");
                    savefig(filename_png);
                    println("Code picture saved at "*filename_png);
                end
                while(!is_shutdown())
                    println("Code generation process completed. please shut down this node.")
                    rossleep(1.0);
                end
            end
            popfirst!(wrapper.schedule);
        end
    else
        #rossleep(0.1);
    end
    logdebug("sender pose callback finished.")
end

@inline function sender_command_cb!(sender_command::Pose, wrapper::ROSWrapperCodeGen)
    logdebug("sender command message received")
    if !(wrapper.started_flag)
        # Got initial measurement in simulation. Starting communication process.
        loginfo("Got initial command.");
        wrapper.started_flag = true;
    else
        #rossleep(1.0);
    end
    logdebug("sender command callback finised")
end;

@inline function get_measurement_schedule_codegen(initial_time::Time,    # Initial ROS time
                                                  num_repeat_max::Int64) # Number of times the trajectory is repeated.
    measurement_schedule = Vector{Tuple{Int64, Time}}(); # Vector of (trajectory_lap, measurement_time)
    push!(measurement_schedule, (1, initial_time + Duration(interval))); # Sender waits interval [s] before starting the communication.
    num_points_in_code = Int(duration/dt) + 1;
    # Add more times to schdule.
    for ii = 1:num_repeat_max
        for jj = 1:num_points_in_code-1
            next_measurement_time = begin
                measurement_schedule[end][2] + Duration(dt)
            end;
            push!(measurement_schedule, (ii, next_measurement_time));
        end
        if ii != num_repeat_max
            next_measurement_time = begin
                # Do not skip first waypoint in codebook
                measurement_schedule[end][2] + Duration(interval)
            end;
            push!(measurement_schedule, (ii+1, next_measurement_time));
        end
    end
    return measurement_schedule
end


node_name = "mbc_codebook_generator";
init_node(node_name, anonymous=true);
# ROS namespace for sender quad
quad_ns_sender = get_param("~quad_ns_sender");
#quad_ns_sender = "quad6"
@assert typeof(quad_ns_sender) == String
# True trajectory class executed by sender. Needed only for post-processing
true_class = get_param("~true_class");
#true_class = 1;
@assert typeof(true_class) == Int && true_class in [ii for ii = 1:length(Codebook_sender)];
# True if sender trajectory is 2D
if has_param("~xz_plane_flag")
    xz_plane_flag = get_param("~xz_plane_flag");
else
    xz_plane_flag = true;
end
@assert typeof(xz_plane_flag) == Bool;
# Number of times the trajectories are repeated
num_repeat_max = get_param("~num_repeat_max");
#num_repeat_max = 3;
@assert typeof(num_repeat_max) == Int && num_repeat_max > 0


# Make sure ROS launch file uses the correct waypoints txt file.
loginfo("quad_ns_sender: $(quad_ns_sender)")
loginfo("true_class: $(true_class)")
loginfo("xz_plane_flag: $(xz_plane_flag)")
loginfo("num_repeat_max: $(num_repeat_max)")

# Make sure ROS launch file uses the correct waypoints txt file.
wrapper = ROSWrapperCodeGen(quad_ns_sender,
                            true_class,
                            xz_plane_flag,
                            num_repeat_max);


loginfo("setup done. waiting for initial pose message...");
spin();
