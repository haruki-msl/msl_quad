/**************************************************************************
  File Name    : pose_track_waypoint_publisher_node.cpp
  Author       : Haruki Nishimura
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : hnishimura@stanford.edu
  Create Time  : Jan 29, 2018.
  Description  : ROS node for waypoint following with pose_track controller.
**************************************************************************/

#include <fstream>
#include <iostream>
#include <string>

#include "mslquad/waypoint_publisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_track_waypoint_publisher");

    // Check if waypoints file is specified.
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);
    if (args.size() != 3) {
        ROS_ERROR("Usage: pose_track_waypoint_publisher_node <quad_ns> <waypoint_file> \n"
                  "Each line should read [waiting_time_s, x_m, y_m, z_m, yaw_deg, gpio_flag]");
        return -1;
    }
    std::string quad_ns = args.at(1);
    std::string waypoints_file_path = args.at(2);

    PX4WaypointPublisher waypoint_publisher(quad_ns, waypoints_file_path);
    ros::spin();
    return 0;
}