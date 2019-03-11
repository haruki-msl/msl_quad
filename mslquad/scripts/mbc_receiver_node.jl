#!/usr/bin/env julia

########################################
## File Name: mbc_receiver_node.jl
## Author: Haruki Nishimura (hnishimura@stanford.edu)
## Date Created: 2019/03/10
## Description: Receiver node setup for active motion-based communication
########################################

#using Distributed
#
#if nprocs() < 2
#    addprocs(1);
#    println("Instantiated a worker process. nprocs() == $(nprocs())");
#end

#@everywhere using ActiveMBC
#@everywhere using POMDPs
#@everywhere using Random
#@everywhere using RobotOS

#@everywhere @rosimport geometry_msgs.msg: Pose, TwistStamped, PoseStamped
#@everywhere rostypegen()
#@everywhere import .geometry_msgs.msg: Pose, TwistStamped, PoseStamped

using ActiveMBC
using JLD
using LinearAlgebra
using POMDPs
using Random
using RobotOS

@rosimport geometry_msgs.msg: Pose, TwistStamped, PoseStamped
rostypegen()
import .geometry_msgs.msg: Pose, TwistStamped, PoseStamped

# Struct for Ros wrapper
mutable struct ROSWrapper
    experiment_id::Int64                         # Experiment ID
    quad_ns::String                              # ROS Namespace for receiver
    true_class::Int64                            # True trajectory class

    command_pub::Publisher{Pose}                 # ROS command publisher
    measurement_sub::Subscriber{TwistStamped}    # ROS measurement subscriber
    receiver_pose_sub::Subscriber{PoseStamped}   # ROS receiver pose subscriber (for groundtruth compuation)
    sender_command_sub::Subscriber{Pose}         # ROS sender command subscriber (for simulation only)

    measurement::Vector{Float64};                # Buffer for latest measurement (of sender's waypoint in Codebook)
    state::Vector{Float64};                      # Buffer for latest receiver state
    initial_time::Time                           # Initial ROS-time when the communiation process started
    schedule::Vector{Tuple{Int64,Time}}          # Vector to store {trajectory_lap, measurement_time} schedule
    num_repeat_max::Int64                        # Maximum number of times sender's trajectory will be repeated (including prior initialization)
    num_repeated::Int64                          # Number of times sender's trajectory has been repeated (including prior initialization)

    simulation_flag::Bool;                       # Flag for ROS-Gazebo simulation using StanfordMSL/mslquad
    started_flag::Bool;                          # Flag for starting communication process
    new_measurement_flag::Bool;                  # Flag for receiving new measurement

    state_history::Vector{Vector{Float64}}       # Receiver's state history
    belief_history::Vector{MHEKFState}           # Receiver's belief history
    action_history::Vector{ControlInput}         # Receiver's control history
    measurement_history::Vector{Vector{Float64}} # Receiver's measurement history

    Codebook::Vector{Vector{Vector{Float64}}}    # Trajectory codebook
    policy::Policy                               # Receiver's policy
    duration::Float64                            # Duration of sender's trajectory
    interval::Float64                            # Interval time between two trajectory executions of sender
    dt::Float64                                  # Interval time between receiver's successive actions

    rng::AbstractRNG                             # Random seed

    min_x::Float64                               # minimum x-value for receiver's position
    max_x::Float64                               # maximum x-value for receiver's position
    min_y::Float64                               # minimum y-value for receiver's position
    max_y::Float64                               # maximum y_value for receiver's position

    function ROSWrapper(experiment_id::Int64,
                        quad_ns::String,
                        true_class::Int64,
                        num_repeat_max::Int64,
                        policy::Policy,
                        rng::AbstractRNG,
                        min_x::Real,
                        max_x::Real,
                        min_y::Real,
                        max_y::Real)
        this = new();
        this.experiment_id = experiment_id;
        this.quad_ns = quad_ns;
        this.true_class = true_class;
        this.started_flag = false;
        this.new_measurement_flag = false;
        this.schedule = Vector{Tuple{Int64,Time}}();
        this.state_history = Vector{Vector{Float64}}();
        this.belief_history = Vector{MHEKFState}();
        this.action_history = Vector{ControlInput}();
        this.measurement_history = Vector{Vector{Float64}}();
        this.Codebook = Codebook; # Global variable
        this.duration = duration; # Global variable
        this.interval = interval; # Global variable
        this.dt = dt;             # Global variable
        this.num_repeat_max = num_repeat_max;
        this.num_repeated = 0;
        this.rng = rng;
        this.policy = policy;
        this.min_x = min_x;
        this.max_x = max_x;
        this.min_y = min_y;
        this.max_y = max_y;
        this.command_pub = Publisher("/"*quad_ns*"/command/pose",
                                     Pose,
                                     queue_size = 1);
        this.measurement_sub = Subscriber("/"*quad_ns*"/camera/camera_measurement",
                                          TwistStamped,
                                          measurement_cb!,
                                          (this, ),
                                          queue_size = 1);
        this.receiver_pose_sub = Subscriber("/"*quad_ns*"/mavros/local_position/pose",
                                           PoseStamped,
                                           receiver_pose_cb!,
                                           (this, ),
                                           queue_size = 1);
        this.simulation_flag = begin
            has_param("/simulation") && get_param("/simulation")
        end;
        if this.simulation_flag
            # For simulation only
            loginfo("Simulation mode");
            this.sender_command_sub = Subscriber("/quad0/command/pose",
                                                 Pose,
                                                 sender_command_cb!,
                                                 (this, ),
                                                 queue_size = 1);
        else
            loginfo("Experiment mode");
        end
        # Pre-compile get_prior
        batch_measurement = rand(2*length(this.Codebook[1]));
        #prior_tmp = remotecall_fetch(get_prior, 2 ,this.Codebook, batch_measurement, this.rng);
        prior_tmp = get_prior(this.Codebook, batch_measurement, this.rng);
        loginfo("get_prior ready.");
        #remotecall_fetch(ekf_update, 2, prior_tmp[1], ControlInput(0.0, 0.0, 0.0, 0.0), rand(2), this.Codebook);
        ekf_update(prior_tmp[1], ControlInput(0.0, 0.0, 0.0, 0.0), rand(2), this.Codebook);

        loginfo("ekf_update ready.")
        #remotecall_fetch(feedback, 2, prior_tmp[1], this.policy);
        feedback(prior_tmp[1], this.policy);
        loginfo("feedback ready.")
        return this
    end
end

function measurement_cb!(measurement::TwistStamped, wrapper::ROSWrapper)
    # println("Measurement message received.");
    logdebug("Measurement message received.");
    # Get measurement schedule from initial measurement time.
    if !(wrapper.started_flag) && !(wrapper.simulation_flag)
        # Got initial measurement. Starting communication process.
        wrapper.initial_time = measurement.header.stamp;
        wrapper.started_flag = true;
        loginfo("Got initial measurement in experiment. Starting communication process at $(to_sec(wrapper.initial_time))");
        wrapper.schedule = begin
            get_measurement_schedule(wrapper.initial_time,
                                     wrapper.num_repeat_max)
        end;
    end
    if wrapper.started_flag
        current_time = measurement.header.stamp;
        if isempty(wrapper.schedule)
            if wrapper.simulation_flag
                filename = joinpath(dirname(pathof(ActiveMBC)), "../result_sim/data_$(wrapper.experiment_id)_$(wrapper.true_class)_$(policy_name).jld");
            else
                filename = joinpath(dirname(pathof(ActiveMBC)), "../result/data_$(wrapper.experiment_id)_$(wrapper.true_class)_$(policy_name).jld");
            end
            save(filename, "experiment_id", wrapper.experiment_id,
                           "true_class", wrapper.true_class,
                           "class_probs", [b.state.prior.p for b in wrapper.belief_history],
                           "log_entropies", [b.entropy for b in wrapper.belief_history],
                           "means", [[bi.μ for bi in b.state.components] for b in wrapper.belief_history],
                           "covs", [[bi.Σ*Matrix(1.0I, size(bi.Σ)) for bi in b.state.components] for b in wrapper.belief_history],
                           "actions", wrapper.action_history,
                           "measurements", wrapper.measurement_history,
                           "states_receiver", wrapper.state_history);
            loginfo("Results saved at "*filename);
            while ! is_shutdown()
                loginfo("Communication process completed. Please shut down.");
                rossleep(1.0);
            end
        end
        current_lap, next_measurement_time = wrapper.schedule[1];
        valid_measurement_time = begin
            current_time >= next_measurement_time - Duration(0.2) &&
            current_time <= next_measurement_time + Duration(0.2);
        end
        if valid_measurement_time
            loginfo("Measurement made at $(to_sec(current_time)) while next_measurement_time is $(to_sec(next_measurement_time))");
            popfirst!(wrapper.schedule);
            wrapper.measurement = [to_sec(current_time),
                                   measurement.twist.linear.x,
                                   measurement.twist.linear.y];
            wrapper.new_measurement_flag = true;
            push!(wrapper.measurement_history, wrapper.measurement);
            if begin current_lap == 1 &&
                     length(wrapper.measurement_history) == length(Codebook[1]) &&
                     all([!isnan(m[2]) for m in wrapper.measurement_history])
                 end;
                # All measurements ready for prior initialization
                wrapper.new_measurement_flag = false;
                loginfo("Initializing prior belief.");
                batch_measurement = vcat([x[2:3] for x in wrapper.measurement_history]...);
                #prior = remotecall_fetch(get_prior,
                #                        2,
                #                        wrapper.Codebook,
                #                        batch_measurement,
                #                        wrapper.rng);
                prior = get_prior(wrapper.Codebook, batch_measurement, wrapper.rng);
                # rossleep(0.5);
                push!(wrapper.state_history, wrapper.state);
                push!(wrapper.belief_history, prior[1]);
                loginfo("Prior ready.");
                for ii = 1:length(wrapper.Codebook)
                    loginfo("Class $(ii) probability: $(wrapper.belief_history[end].state.prior.p[ii])");
                end
            end
            if current_lap > 1 && length(wrapper.belief_history) == 0
                logerr("Prior not initialized properly! Most likely some observation was not made.");
            end
        end
    else
        rossleep(0.01);
    end
    #println("Measurement message ended.");
    logdebug("Measurement message ended.");
end;

function receiver_pose_cb!(receiver_pose::PoseStamped, wrapper::ROSWrapper)
    logdebug("Pose message received.");
    r = stateVecFromPose(receiver_pose.pose);
    wrapper.state = r;
    current_time = receiver_pose.header.stamp;
    if wrapper.started_flag
        if isempty(wrapper.schedule)
            if wrapper.simulation_flag
                filename = joinpath(dirname(pathof(ActiveMBC)), "../result_sim/data_$(wrapper.experiment_id)_$(wrapper.true_class)_$(policy_name).jld");
            else
                filename = joinpath(dirname(pathof(ActiveMBC)), "../result/data_$(wrapper.experiment_id)_$(wrapper.true_class)_$(policy_name).jld");
            end
            save(filename, "experiment_id", wrapper.experiment_id,
                           "true_class", wrapper.true_class,
                           "class_probs", [b.state.prior.p for b in wrapper.belief_history],
                           "log_entropies", [b.entropy for b in wrapper.belief_history],
                           "means", [[bi.μ for bi in b.state.components] for b in wrapper.belief_history],
                           "covs", [[bi.Σ*Matrix(1.0I, size(bi.Σ)) for bi in b.state.components] for b in wrapper.belief_history],
                           "actions", wrapper.action_history,
                           "measurements", wrapper.measurement_history,
                           "states_receiver", wrapper.state_history);
            loginfo("Results saved at "*filename);
            while ! is_shutdown()
                loginfo("Communication process finished. Please shut down.")
                rossleep(1.0);
            end
        end
        ~, next_measurement_time = wrapper.schedule[1]
        if next_measurement_time  + Duration(0.2) < current_time
            popfirst!(wrapper.schedule);
            logwarn("Measurement schedule popped in receiver_pose_cb. Likely a measurement was missed.")
            wrapper.measurement = [to_sec(current_time), NaN, NaN];
            push!(wrapper.measurement_history, wrapper.measurement);
            wrapper.new_measurement_flag = true;
        end
        if begin wrapper.new_measurement_flag &&
                 length(wrapper.belief_history) > 0 end
            # New measurement ready, prior ready: control receiver!
            control(wrapper);
            # Turn off measurement flag;
            wrapper.new_measurement_flag = false;
        end
    else
        rossleep(0.01);
    end
    logdebug("Pose message ended.");
end;

function control(wrapper::ROSWrapper)
    # Don't execute this function if new measurement is not available
    # or prior not available.
    @assert begin wrapper.new_measurement_flag &&
                  length(wrapper.belief_history) > 0 end;
    push!(wrapper.state_history, wrapper.state);
    prev_belief = wrapper.belief_history[end];
    prev_state = wrapper.state;
    if isempty(wrapper.action_history);
        prev_action = ControlInput(0.0, 0.0, 0.0, 0.0);
    else
        prev_action = wrapper.action_history[end];
    end
    new_measurement = wrapper.measurement;
    #new_belief = remotecall_fetch(ekf_update, 2, prev_belief, prev_action, new_measurement[2:3], wrapper.Codebook);
    new_belief = ekf_update(prev_belief, prev_action, new_measurement[2:3], wrapper.Codebook);
    push!(wrapper.belief_history, new_belief);
    loginfo("belief updated.");
    #new_action = remotecall_fetch(feedback, 2, new_belief, wrapper.policy);
    new_action = feedback(new_belief, wrapper.policy);
    push!(wrapper.action_history, new_action);
    loginfo("new control command computed: $(new_action)");
    # In state_trans, timestep and true_class can be arbitrary since we are not using sender's position.
    # TODO: Avoid using state_trans to just update the receiver's state.
    new_commanded_state = state_trans([prev_state;wrapper.Codebook[1][1]], new_action, wrapper.Codebook, 1, 1)[1:6];
    p_tmp = poseFromStateVecCalibrated(new_commanded_state);
    # Check if new pose is within [min_x, max_x]x[min_y, max_y]
    try
        @assert begin wrapper.min_x < p_tmp.position.x < wrapper.max_x &&
                      wrapper.min_y < p_tmp.position.y < wrapper.max_y end;
    catch
        while ! is_shutdown()
            logerr("receiver is going out of workspace bounds! Please shut down.")
            rossleep(1.0);
        end
    end
    # TODO: Clean-up this messy p_tmp thing.
    p = Pose();
    p.position.x, p.position.y, p.position.z = p_tmp.position.x, p_tmp.position.y, p_tmp.position.z;
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = p_tmp.orientation.w, p_tmp.orientation.x, p_tmp.orientation.y, p_tmp.orientation.z;
    # Publish surely.
    for ii = 1:10
        publish(wrapper.command_pub, p);
    end
    loginfo("pose command message published.")
end

# For simulation only
function sender_command_cb!(sender_command::Pose, wrapper::ROSWrapper)
    logdebug("Sender command message received.")
    if wrapper.simulation_flag && !(wrapper.started_flag)
        # Got initial measurement in simulation. Starting communication process.
        wrapper.initial_time = get_rostime();
        loginfo("Got initial measurement in simulation. Starting communication process at $(to_sec(wrapper.initial_time))");
        wrapper.schedule = begin
            get_measurement_schedule(wrapper.initial_time,
                                     wrapper.num_repeat_max)
        end;
        wrapper.started_flag = true;
    else
        rossleep(1.0);
    end
    logdebug("Sender command message ended.");
end;


node_name = "mbc_receive_controller";
init_node(node_name, anonymous=true);
# ROS namespace for receiver quad
quad_ns_receiver = get_param("~quad_ns_receiver");
#quad_ns_receiver = "quad1"
@assert typeof(quad_ns_receiver) == String
# True trajectory class executed by sender. Needed only for post-processing
true_class = get_param("~true_class");
#true_class = 1;
@assert typeof(true_class) == Int && true_class in [ii for ii = 1:length(Codebook_sender)];
# Number of times the trajectories are repeated
num_repeat_max = get_param("~num_repeat_max");
#num_repeat_max = 3;
@assert typeof(num_repeat_max) == Int && num_repeat_max > 0
# Policy of receiver
policy_name = get_param("~policy");
#policy_name = "mcts"
@assert typeof(policy_name) == String && policy_name in ["mcts", "greedy", "random"];
# Workspace bounds
min_x, max_x = get_param("~min_x"), get_param("~max_x");
min_y, max_y = get_param("~min_y"), get_param("~max_y");
#min_x, max_x = -7, 7;
#min_y, max_y = -2.7, 2.7;
@assert typeof(min_x) <: Real
@assert typeof(max_x) <: Real
@assert typeof(min_y) <: Real
@assert typeof(max_y) <: Real
# Experiment ID
experiment_id = get_param("~experiment_id");
#experiment_id = 1;
@assert typeof(experiment_id) == Int && experiment_id > 0

rng = MersenneTwister(123);
if policy_name == "mcts"
    policy = mcts_policy(rng);
elseif policy_name == "greedy"
    policy = GreedyPlanner(rng);
elseif policy_name == "random"
    policy = RandomPlanner(rng);
else
    logerr("$(policy_name) is not a valid policy name!");
end
# Make sure ROS launch file uses the correct waypoints txt file.
loginfo("experiment_id: $(experiment_id)")
loginfo("quad_ns_receiver: $(quad_ns_receiver)")
loginfo("true_class: $(true_class)")
loginfo("num_repeat_max: $(num_repeat_max)")
loginfo("policy: $(policy_name)")
wrapper = ROSWrapper(experiment_id,
                     quad_ns_receiver,
                     true_class,
                     num_repeat_max,
                     policy,
                     rng,
                     min_x, max_x, min_y, max_y);
loginfo("setup done. spinning...");
spin();
