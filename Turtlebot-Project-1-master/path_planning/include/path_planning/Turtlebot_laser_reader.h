// tutrtlebot_laser_reader.h
// 
// Subscribe to the "/scan" (laser data from sensor)
// it is responsible to save the range data in hashmap 
// it will find the minimum angle among all directions
// Finished by Group: RoboMaster (06/10/2021)

#include <ros/ros.h>
#include <iostream>
#include <unordered_map>
#include <sensor_msgs/LaserScan.h>

#ifndef __TURTLEBOT_LASER_READER_H
#define __TURTLEBOT_LASER_READER_H

// --------------------------------- Turtlebot_laser_reader --------------------------------
// This module is responsable to communicate with laser on the turtlebot and save the data
class Turtlebot_laser_reader
{
    public:
        // method
        float laser_read_range(int angle);      // return the sacn data at required angle
        float get_smallest_range();             // return the smallest range 
        int get_angle_to_smallest_range();      // return the angle for the smallest range

        // constructor and destructor
        Turtlebot_laser_reader(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~Turtlebot_laser_reader();

    private:
        // data
        std::unordered_map<int, float> _scan_data;  // hashmap saves all ranges using angle as key
        float _range_min;                           // minimum range
        int _angle_min;                             // angle at minimum range

        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        // ROS Topic Subscribers
        ros::Subscriber _laser_scan_sub;

        // method
        // update scan data & find the minimum range and its related angle
        void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    };
#endif

