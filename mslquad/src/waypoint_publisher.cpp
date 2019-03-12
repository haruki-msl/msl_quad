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
#include <ros/static_assert.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

#include "mslquad/waypoint_publisher.h"

PX4WaypointPublisher::PX4WaypointPublisher(const std::string& quad_ns, const std::string& waypoints_file_path) :
        quadNS_("/" + quad_ns + "/"),
        waypoints_file_(waypoints_file_path){

    // retrieve ROS parameters
    const std::string control_freq_param = quadNS_ + "default_controller/control_freq";
    const std::string pose_target_param = quadNS_ + "default_controller/pose_target_topic";
    const std::string gpio_pin_param = quadNS_ + "gpio/gpio_pin_number";
    const std::string gpio_target_param = quadNS_ + "gpio/gpio_target_topic";
    ROS_ASSERT_MSG(ros::param::has(control_freq_param) &&
                   ros::param::has(pose_target_param),
                   "Required ROS param(s) not found in ROS parameter server. Did you specify correct mav name?");
    if (!ros::param::has(gpio_pin_param) || !ros::param::has(gpio_target_param)) {
        ROS_WARN("GPIO ROS param(s) not found in ROS parameter server. GPIO control will not work properly.");
    }
    ros::param::get(control_freq_param, maxFreq_);
    std::string poseTargetTopic, gpioTargetTopic;
    ros::param::get(pose_target_param, poseTargetTopic);
    int pin_number;
    ros::param::get(gpio_pin_param, pin_number);
    ros::param::get(gpio_target_param, gpioTargetTopic);

    // ROS publisher
    px4WayPointsPub_ = nh_.advertise<geometry_msgs::Pose>(quadNS_ + poseTargetTopic, 1);
    odroidXU4GpioPub_ = nh_.advertise<std_msgs::Bool>(quadNS_ + gpioTargetTopic, 1);

    // Open file containing waypoints info
    std::ifstream waypoints_file(waypoints_file_path);
    if (waypoints_file.is_open()) {
        double highest_waypoint_frequency = 0.0;
        double total_duration_s = 0.0;
        double duration_s, x, y, z, yaw_deg;
        int gpio_out = 0;
        // Only read complete waypoints
        while (waypoints_file >> duration_s >> x >> y >> z >> yaw_deg >> gpio_out) {
            geometry_msgs::Point position;
            position.x = x;
            position.y = y;
            position.z = z;
            ros::Duration waiting_time_accumulated_s(total_duration_s);
            WaypointWithTime waypoint {position, yaw_deg, waiting_time_accumulated_s, static_cast<bool>(gpio_out)};
            waypoints_.push(waypoint);
            total_duration_s += duration_s;
            // Check waypoint update frequency
            double waypoint_frequency = 1.0/duration_s;
            if (waypoint_frequency > highest_waypoint_frequency) {
                highest_waypoint_frequency = waypoint_frequency;
            }
        }
        ros::Duration(1.0).sleep();
        ROS_ASSERT_MSG(highest_waypoint_frequency <= maxFreq_,
                       "Waypoint update frequency %.2fHz too high to be published. Maximum: %.2fHz",
                       highest_waypoint_frequency,
                       maxFreq_);
        waypoints_file.close();
        ROS_INFO("Read %d waypoints. Duration: %.2f[s]", static_cast<int>(waypoints_.size()), total_duration_s);
    } else {
        ROS_ERROR_STREAM("Unable to open waypoints file: " << waypoints_file_path);
    }

    // Record initial time
    ROS_INFO("Initiated Waypoint Publisher node. Frequency: %.2f Hz", maxFreq_);

    // Start timer
    initial_time_ = ros::Time::now();
    timer_ = nh_.createTimer(
            ros::Duration(1.0/maxFreq_),
            &PX4WaypointPublisher::timerCB ,this);
}

void PX4WaypointPublisher::timerCB(const ros::TimerEvent& event) {
    current_time_ = ros::Time::now();
    if (waypoints_.empty()) {
        ROS_INFO("Finished publishing waypoints. Closing...");
        ros::shutdown();
    } else if (current_time_ - initial_time_ >= waypoints_.front().waiting_time_accumulated_s) {
        WaypointWithTime new_point = waypoints_.front();
        waypoints_.pop();
        // Pop all the points that are already missed (,which will not happen theoretically).
        while (!waypoints_.empty() && current_time_ - initial_time_ > waypoints_.front().waiting_time_accumulated_s) {
            new_point = waypoints_.front();
            waypoints_.pop();
        }
        geometry_msgs::Pose target_pose;
        // Publish GPIO command.
        std_msgs::Bool gpio_out;
        gpio_out.data = static_cast<unsigned char>(new_point.gpio_out);
        for (int ii = 0; ii < 10; ++ii) {
            odroidXU4GpioPub_.publish(gpio_out); // TODO: Change GPIO control from topic to service communication.
        }
        // Publish Waypoint.
        target_pose.position = new_point.position;
        tf::Quaternion q = tf::createQuaternionFromYaw(new_point.yaw_deg * M_PI/180.0);
        tf::quaternionTFToMsg(q, target_pose.orientation);
        px4WayPointsPub_.publish(target_pose);
        ROS_INFO("Waypoint Published. Position[m]: [%.2f, %.2f, %.2f], Yaw[deg]: %.2f, GPIO: %d",
                 new_point.position.x,
                 new_point.position.y,
                 new_point.position.z,
                 new_point.yaw_deg,
                 new_point.gpio_out);
    }
}