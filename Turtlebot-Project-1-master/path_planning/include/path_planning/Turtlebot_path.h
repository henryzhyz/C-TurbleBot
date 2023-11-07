// turtlebot_path.h
// 
// Create a publisher named "/path" to update the robot's trajactory simultaneously.
// It subscribes to "/odom" which records the data from motion sensor. 
// Finished by Group: RoboMaster (06/10/2021)

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#ifndef __TURTLEBOT_PATH_H
#define __TURTLEBOT_PATH_H

// --------------------------------- Turtlebot_path --------------------------------
// This module is responsable to output the trajectory of the robot
class Turtlebot_path
{
    private:

        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        // ROS Topic Subscribers
        ros::Subscriber _odom_sub;

        // ROS Topic Publisher
        nav_msgs::Path path;
        ros::Publisher _path_pub;

    public:

        // constructor and destructor
        Turtlebot_path(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~Turtlebot_path();

        // methods
        // publish the trajectory following by the robot by subscribing the reading meassage from odometry.
        void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif