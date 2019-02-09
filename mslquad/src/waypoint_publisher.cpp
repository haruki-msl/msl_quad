/**************************************************************************
  File Name    : waypoint_publisher.cpp
  Author       : Haruki Nishimura
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : hnishimura@stanford.edu
  Create Time  : Jan 29, 2018.
  Description  : waypoint (position + yaw + waiting time) publisher.
**************************************************************************/
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <queue>

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "mslquad/waypoint_publisher.h"

PX4WaypointPublisher::PX4WaypointPublisher(const std::string& quad_ns, const std::string& waypoints_file_path) :
        quadNS_("/" + quad_ns + "/"),
        waypoints_file_(waypoints_file_path){

    // retrieve ROS parameters
    ros::param::get(quadNS_ + "default_controller/slow_freq", maxFreq_);
    std::string poseTargetTopic;
    ros::param::get(quadNS_ + "default_controller/pose_target_topic", poseTargetTopic);

    // ROS publisher
    px4WayPointsPub_ = nh_.advertise<geometry_msgs::Pose>(quadNS_ + poseTargetTopic, 1);

    // Open file containing waypoints info
    std::ifstream waypoints_file(waypoints_file_path);
    if (waypoints_file.is_open()) {
        double highest_waypoint_frequency = 0.0;
        double total_duration_s = 0.0;
        double duration_s, x, y, z, yaw_deg;
        // Only read complete waypoints
        while (waypoints_file >> duration_s >> x >> y >> z >> yaw_deg) {
            total_duration_s += duration_s;
            geometry_msgs::Point position;
            position.x = x;
            position.y = y;
            position.z = z;
            ros::Duration waiting_time_s(duration_s);
            WaypointWithTime waypoint {position, yaw_deg, waiting_time_s};
            waypoints_.push(waypoint);
            // Check waypoint update frequency
            double waypoint_frequency = 1.0/duration_s;
            if (waypoint_frequency > highest_waypoint_frequency) {
                highest_waypoint_frequency = waypoint_frequency;
            }
        }
        ROS_ASSERT_MSG(highest_waypoint_frequency <= maxFreq_,
                       "Waypoint update frequency %.2fHz too high to be published. Maximum: %.2fHz",
                       highest_waypoint_frequency,
                       maxFreq_);
        waypoints_file.close();
        ROS_INFO("Read %d waypoints. Duration: %.2f[s]", (int) waypoints_.size(), total_duration_s);
    } else {
        ROS_ERROR_STREAM("Unable to open waypoints file: " << waypoints_file_path);
    }

    // Record initial time
    initial_time_ = ros::Time::now();
    last_action_time_ = initial_time_;
    ROS_INFO("Waypoint publisher node initiated. Frequency: %.2f Hz", maxFreq_);

    // Start timer
    slowTimer_ = nh_.createTimer(
            ros::Duration(1.0/maxFreq_),
            &PX4WaypointPublisher::slowTimerCB ,this);
}

void PX4WaypointPublisher::slowTimerCB(const ros::TimerEvent& event) {
    current_time_ = ros::Time::now();
    ros::Duration time_elapsed_s = current_time_ - last_action_time_;
    WaypointWithTime& new_point = waypoints_.front();
    // Only publish a new waypoint after the waiting time.
    if (time_elapsed_s > new_point.waiting_time_s && !waypoints_.empty()) {
        geometry_msgs::Pose target_pose;
        target_pose.position = new_point.position;
        tf::Quaternion q = tf::createQuaternionFromYaw(new_point.yaw_deg * M_PI/180.0);
        tf::quaternionTFToMsg(q, target_pose.orientation);
        px4WayPointsPub_.publish(target_pose);
        last_action_time_ = current_time_;
        ROS_INFO("Waypoint Published. Position[m]: [%.2f, %.2f, %.2f], Yaw[deg]: %.2f",
                 new_point.position.x,
                 new_point.position.y,
                 new_point.position.z,
                 new_point.yaw_deg);
        waypoints_.pop();
    }
    if (waypoints_.empty()) {
        ROS_INFO("Finished publishing waypoints. Closing...");
        ros::shutdown();
    }
}