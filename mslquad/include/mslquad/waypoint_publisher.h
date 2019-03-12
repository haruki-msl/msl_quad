/**************************************************************************
  File Name    : waypoint_publisher.h
  Author       : Haruki Nishimura
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : hnishimura@stanford.edu
  Create Time  : Jan 29, 2018.
  Description  : waypoint (position + yaw + waiting time) publisher.
**************************************************************************/

#ifndef __POSE_TRACK_WAYPOINT_PUBLISHER_H__
#define __POSE_TRACK_WAYPOINT_PUBLISHER_H__

#include <string>
#include <queue>

#include <ros/ros.h>

#include "geometry_msgs/Point.h"

class PX4WaypointPublisher {
public:
    PX4WaypointPublisher(const std::string& quad_ns, const std::string& waypoints_file_path);

    struct WaypointWithTime {
        geometry_msgs::Point position;
        double yaw_deg = 0.0;
        ros::Duration waiting_time_accumulated_s;
        bool gpio_out = false;
    };

protected:
    std::string quadNS_; // ROS name space
    ros::NodeHandle nh_;
    ros::Publisher px4WayPointsPub_;
    ros::Publisher odroidXU4GpioPub_;

private:
    std::string waypoints_file_;
    std::queue<WaypointWithTime> waypoints_;
    double maxFreq_;
    ros::Time initial_time_;
    ros::Time current_time_;
    ros::Timer timer_;

    void timerCB(const ros::TimerEvent& event);
};

#endif //__POSE_TRACK_WAYPOINT_PUBLISHER_H__
