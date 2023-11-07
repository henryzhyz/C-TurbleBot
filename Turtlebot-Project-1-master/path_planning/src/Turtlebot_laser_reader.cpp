// tutrtlebot_laser_reader.cpp
// This module is responsable to communicate with laser on the turtlebot and save the data
// Finished by Group: RoboMaster (06/10/2021)

#include "path_planning/Turtlebot_laser_reader.h"
#include <assert.h>     /* assert */

// constructor
// subscribe to the "/scan" and start to read laser data
// Input:               
//              nh: current node
//              nh_priv: previous node
Turtlebot_laser_reader::Turtlebot_laser_reader(ros::NodeHandle nh, ros::NodeHandle nh_priv)
: _nh(nh),
_nh_priv(nh_priv)
{
    _laser_scan_sub = nh.subscribe( "/scan", 1000, & Turtlebot_laser_reader::laserScanMsgCallBack, this );
}

// destructor
Turtlebot_laser_reader::~Turtlebot_laser_reader()
{
}

// update the data saved in the hashmap
// find the the minimum range and its related angle
// Input:               
//              msg: a reference to the received/updated data from laser scan
void Turtlebot_laser_reader::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg) 
{
    // ensure the msg has the right size. Otherwise, raise error message.
    assert(msg->ranges.size()==360);

    // get all the distance at each angle and search the minimum angle and range
    _range_min =  msg->range_max;
    _angle_min = 0;

    for (int i = 0; i < msg->ranges.size(); i++)
    {
        _scan_data[i] = msg->ranges.at(i);
        if (_range_min > _scan_data[i])
        {
            _angle_min = i;
            _range_min = _scan_data[i];
            
        }
    }
    // Message for dubugging
    // ROS_INFO("size: %d, angle min: %d distance min: %f", msg->ranges.size(), _angle_min, _range_min);
    // ensure the minimum angle has been updated
    assert(_angle_min != -1);
}

// return the smallest range
// return:
//              a float number of smallest range among al directions
float Turtlebot_laser_reader::get_smallest_range()
{
    return _range_min;
}

// return th angle ath which the range is the smallest
// return:
//              an integer of angle where has the smallest range among all directions
int Turtlebot_laser_reader::get_angle_to_smallest_range()
{
    return _angle_min;
}

// return the ranged for selected angle
// Input:               
//              angle: a integer between 0-359. It indicates range direction.
// return:
//              a float number of range at specific angle
float Turtlebot_laser_reader::laser_read_range(int angle)
{
    return _scan_data[angle];
}