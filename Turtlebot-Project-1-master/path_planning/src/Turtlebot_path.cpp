// turtlebot_path.cpp
// Finished by Group: RoboMaster (06/10/2021)

#include "path_planning/Turtlebot_path.h"

// constructor
Turtlebot_path::Turtlebot_path(ros::NodeHandle nh, ros::NodeHandle nh_priv)
{
    _nh = nh;
    _nh_priv = nh_priv;

    // register a message will be published on the topic 'trajectory'
    _path_pub = _nh.advertise<nav_msgs::Path>("trajactory", 10, true);

    // subscribe to the topic 'odom' with the master
    _odom_sub = _nh.subscribe<nav_msgs::Odometry>("odom", 10, &Turtlebot_path::odomMsgCallBack, this);
}

// destructor
Turtlebot_path::~Turtlebot_path()
{
}

// publish the trajectory following by the robot by subscribing the reading meassage from odometry
void Turtlebot_path::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    // get the current position by subscribing the topic 'odom'
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
    this_pose_stamped.pose.position.y = msg->pose.pose.position.y;
    this_pose_stamped.pose.position.z = msg->pose.pose.position.z;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "odom";

    // insert the current position in the message 'path'
    path.poses.push_back(this_pose_stamped);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";

    // publish the current position on topic 'trajactory'
    _path_pub.publish(path);   
}