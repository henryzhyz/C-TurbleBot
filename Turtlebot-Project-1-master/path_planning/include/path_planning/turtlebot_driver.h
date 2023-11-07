// tutrtlebot_driver.h
// 
// Subscribe to the "/twist" (angular and linear velocity)
// it is responsible to control the turtlbot 
// it will 4 different mode: turn left/right; move forward; stop
// Finished by Group: RoboMaster (06/10/2021)

#ifndef __TURTLEBOT_DRIVER_H
#define __TURTLEBOT_DRIVER_H

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

#define LINEAR_VELOCITY  0.1           // linear velocity
#define ANGULAR_VELOCITY 0.1           // angular velocity

// --------------------------------- Turtlebot_driver --------------------------------
// This module is responsable to control the dynamics/movement of the turtlebot 
class Turtlebot_driver
{
    public:
        // constructor and destructor
        Turtlebot_driver(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~Turtlebot_driver();

        // method
        bool init();                        // init the publisher and ros parameter 
        void move_forward();                // move the turtlebot forward
        void turn_left();                   // turn the turtlebot left
        void turn_right();                  // turn the turtlebot right
        void stop();                        // stop the vehicle

    private:
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        // ROS Topic Publishers
        ros::Publisher _cmd_vel_pub;

        // Function prototypes
        void updatecommandVelocity(double linear, double angular);
};
#endif // TURTLEBOT_DRIVE_H_