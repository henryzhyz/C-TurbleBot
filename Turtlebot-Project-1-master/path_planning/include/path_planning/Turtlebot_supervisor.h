// tutrtlebot_supervisor.h
// 
// responsible to make the decision based on the laser reader and it own position
// read data from laser module
// make a decision based on current condition
// send the decision to the driver/motor
// initialize the path module to publish trajectory
// Finished by Group: RoboMaster (06/10/2021)

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <cstdlib>
#include "path_planning/Turtlebot_path.h"
#include "path_planning/turtlebot_driver.h"
#include "path_planning/Turtlebot_laser_reader.h"

#ifndef __TURTLEBOT_SUPERVISOR_H
#define __TURTLEBOT_SUPERVISOR_H

// --------------------------------- Turtlebot_supervisor --------------------------------
// This module is responsable to drive the turtlebot based on the wall
class Turtlebot_supervisor
{
    public:
        // constructor and destructor
        Turtlebot_supervisor(ros::NodeHandle nh, ros::NodeHandle nh_priv); 
        ~Turtlebot_supervisor();

        // method
        void publish_command(void);                  // publish commad for execution
        void make_decision(void);                    // make the move decision based on laseer data

    private:
        // enum
        // enum defining the types of possible move
        enum _decision_type                          // turn left: turtlebot should rotate in CCW direction (no linear velocity)
        {                                            // turn right: turtlebot should rotate in CW direction (no linear velocity)
            turn_left, turn_right, move_forward      // move forward: turtlebot should move forward with default speed
        };

        // data
        Turtlebot_driver* _my_driver;                // a pointer to the driver module
        Turtlebot_laser_reader* _my_laser_reader;    // a pointer to the laser scanner module 
        Turtlebot_path* _my_path;                    // a pointer to the trajectory module
        _decision_type _my_decision;                 // a varible to save the current moving decision

        // Constant Variables
        const float _far_distance = 0.3;              // the treshold distance where the robot is defined as far from the wall
        const float _close_distance = 0.2;            // the treshold distance where the robot is defined as close to the wall
        const int _angle_threshold = 8;               // the treshold angle (degree) where the direction is considered as close to required angle.
        const float _threshold_distance = 0.01;      // maximum allowable difference for two same distance.

        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;
};
#endif