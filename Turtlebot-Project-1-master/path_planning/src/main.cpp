// main.cpp
// This file is responsible to create a node for path planning
// 
// Finished by Group: RoboMaster (06/10/2021)

#include "ros/ros.h"
#include "path_planning/Turtlebot_supervisor.h"

int main( int argc, char** argv )
{
    // Initialize the ros
    ros::init( argc, argv, "path_planning" );
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv( "~" );

    // create a supervisor
    Turtlebot_supervisor my_supervisor(nh, nh_priv);

    // keep planning the path
    ros::Rate loop_rate( 30 );
    while( ros::ok() )
    { 
        // make decision and publish the decision
        my_supervisor.make_decision();
        my_supervisor.publish_command();
        
        // ros spin
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
