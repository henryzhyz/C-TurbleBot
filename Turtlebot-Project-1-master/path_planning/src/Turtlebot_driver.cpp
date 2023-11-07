// tutrtlebot_driver.cpp
// This module is responsable to control the dynamics/movement of the turtlebot 
// Finished by Group: RoboMaster (06/10/2021)

#include "path_planning/turtlebot_driver.h"

// constructor
// init gazebo ros turtlebot3 node
// Input:               
//              nh: current node
//              nh_priv: previous node
Turtlebot_driver::Turtlebot_driver(ros::NodeHandle nh, ros::NodeHandle nh_priv)
: _nh(nh),
_nh_priv(nh_priv)
{
    //Init gazebo ros turtlebot3 node
    auto ret = init();
    ROS_ASSERT(ret);
}

// destructor
// stop the turtlebot
Turtlebot_driver::~Turtlebot_driver()
{
    updatecommandVelocity(0.0, 0.0);
    // ros::shutdown();
}

// init the publisher and ros parameter 
// Return:
//                  a boolean indicates the state of initialization
bool Turtlebot_driver::init()
{
    // initialize ROS parameter
    std::string cmd_vel_topic_name = _nh.param<std::string>("cmd_vel_topic_name", "");

    // initialize publishers
    _cmd_vel_pub   = _nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

    return true;
}

// publish the velocity 
// input:
//      linear,angular: the linear and angular velocity of the turtlobot 
void Turtlebot_driver::updatecommandVelocity(double linear, double angular)
{
    geometry_msgs::Twist cmd_vel;
    
    // set the velocity 
    cmd_vel.linear.x  = linear;
    cmd_vel.angular.z = angular;

    _cmd_vel_pub.publish(cmd_vel);
}

// move the turtlebot forward with default speed
void Turtlebot_driver::move_forward()
{
    updatecommandVelocity(LINEAR_VELOCITY, 0.0);
}

// turn the robot left 
void Turtlebot_driver::turn_left()
{
    updatecommandVelocity(ANGULAR_VELOCITY*radius, ANGULAR_VELOCITY);
}

// turn the robot right
void Turtlebot_driver::turn_right()
{
    updatecommandVelocity(ANGULAR_VELOCITY*radius, -1 * ANGULAR_VELOCITY);
}

// stop the turtlebot 
void Turtlebot_driver::stop()
{
    updatecommandVelocity(0.0, 0.0);
}

