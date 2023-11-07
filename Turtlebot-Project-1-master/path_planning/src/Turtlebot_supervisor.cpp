// tutrtlebot_supervisor.cpp
// This module is responsable to drive the turtlebot based on the wall
// Finished by Group: RoboMaster (06/10/2021)

#include "path_planning/Turtlebot_supervisor.h"

// constructor
// create three object for turtlebot driver, laser reader, and trajectory indicator
// initialize the value of private variable
// Input:               
//              nh: current node
//              nh_priv: previous node
Turtlebot_supervisor::Turtlebot_supervisor(ros::NodeHandle nh, ros::NodeHandle nh_priv)
{
    // handle ros node
    _nh = nh;
    _nh_priv = nh_priv;
    ROS_INFO("TurtleBot3 Simulation Node Init");

    // create a driver, a laser reader, and a trajectory indicator
    _my_driver = new Turtlebot_driver(_nh, _nh_priv);
    _my_laser_reader = new Turtlebot_laser_reader(_nh, _nh_priv);
    _my_path = new Turtlebot_path(_nh, _nh_priv);

    // initialize variables
    _my_decision = move_forward;
    _my_driver->stop(); 
}


// destructor
// delete the created objects
Turtlebot_supervisor::~Turtlebot_supervisor()
{
    delete _my_driver;
    delete _my_laser_reader;
    delete _my_path;
    ros::shutdown();
}

// send the command to the driver to move the turtlebot
// the decision comes from the private variable _my_decision
void Turtlebot_supervisor::publish_command(void)
{

    switch (_my_decision)
    {
    case turn_right:
        // Debug: 
        // ROS_INFO("Turn right");

        _my_driver->turn_right();
 
        break;

    case turn_left:
        // Debug: 
        // ROS_INFO("Turn left");

        _my_driver->turn_left();
 
        break;

    case move_forward:
        // Debug: 
        // ROS_INFO("Turn right");

        _my_driver->move_forward());
 
        break;

    default:
        // Not expected, warning
        ROS_WARN("Wrong decision");
        break;
    }
}

// decide which strategy to take based on current position
// the result is saved in _my_decision
void Turtlebot_supervisor::make_decision(void)
{
    // get the range and angle
    float min_range = _my_laser_reader->get_smallest_range();
    int min_angle = _my_laser_reader->get_angle_to_smallest_range();

    // default decision
    _my_decision = move_forward;

    // too far away from wall
    if (min_range > _far_distance)
    {
        // Debug:
        // ROS_INFO("Too far");

        // minimum range at angle around 80 degree
        if (std::abs(min_angle - 80) < _angle_threshold || std::abs(_my_laser_reader->laser_read_range(80) - min_range) < _threshold_distance)
        {
            _my_decision = move_forward;
        }
        else
        {
            if (min_angle < 260 && min_angle > 80)
            {
                _my_decision = turn_left;
            }
            else
            {
                _my_decision = turn_right;
            }
        }
        return;
    }

    // too close to the wall
    if (min_range < _close_distance)
    {
        // Debug:
        // ROS_INFO("Too close");

        // minimum range at angle around 100 degree
        if (std::abs(min_angle - 100) < _angle_threshold || std::abs(_my_laser_reader->laser_read_range(100) - min_range) < _threshold_distance)
        {
            _my_decision = move_forward;
        }
        else
        {
            if (min_angle < 280 && min_angle > 100)
            {
                _my_decision = turn_left;
            }
            else
            {
                _my_decision = turn_right;
            }
        }
        return;
    }

    // normal distance
    // make sure the one one the left is closest
    if (std::abs(min_range - (_far_distance + _close_distance)/2) < _threshold_distance)
    {
        // Debug:
        // ROS_INFO("follow");
        
        // parallel to the wall
        // minimum range at around 90 degree
        if (std::abs(min_angle - 90) < (_angle_threshold/4) || std::abs(_my_laser_reader->laser_read_range(90) - min_range) < _threshold_distance)
        {
            _my_decision = move_forward;
        }
        else
        {
            if (min_angle < 270 && min_angle > 90)
            {
                _my_decision = turn_left;
            }
            else
            {
                _my_decision = turn_right;
            }
        }
        return;
    }
}