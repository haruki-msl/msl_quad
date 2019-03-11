#!/usr/bin/env julia

########################################
## File Name: mbc_generate_sender_waypoints.jl
## Author: Haruki Nishimura (hnishimura@stanford.edu)
## Date Created: 2019/03/10
## Description: generate motion-based communication waypoints file for sender
########################################

using ActiveMBC
using DelimitedFiles
using RobotOS

init_node("mbc_sender_waypoints_generator", anonymous=true)
# True trajectory class executed by sender
true_class = get_param("~true_class");
# Number of times the trajectories are repeated
num_repeat_max = get_param("~num_repeat_max");
@assert typeof(true_class) == Int && true_class in [ii for ii = 1:length(Codebook_sender)];
@assert typeof(num_repeat_max) == Int && num_repeat_max > 0;

filename = joinpath((@__DIR__), "..", "resource/mbc_waypoints_sender_$(true_class).txt");
TmpArray = Vector{String}();
for jj = 1:num_repeat_max;
    for kk = 1:length(Codebook_sender[true_class])
        TmpString = vcat(string.(Codebook_sender[true_class][kk][1:end]),
                                 "1");
        push!(TmpArray, join(TmpString, " "));
    end
    if jj < num_repeat_max
        TmpString = vcat(string(interval),
                         string.(Codebook_sender[true_class][1][2:end]),
                         "0");
        push!(TmpArray, join(TmpString, " "));
    end
end
open(filename, "w") do io
    writedlm(io, TmpArray)
end;
@assert sum([parse(Float64, TmpString[1:3]) for TmpString in TmpArray]) ≈ begin
    duration*num_repeat_max + interval*(num_repeat_max - 1)
end;
loginfo("Sender waypoints generation done.")
