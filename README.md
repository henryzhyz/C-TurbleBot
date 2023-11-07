# Cplusplus-TurbleBot
# MTRX3760-Project-1
Groupg Name: RoboMaster
Team Member: 480069009, 490149261,490189050, 490440906

#####################################

# Environment set up
## make a directory
mkdir -p turtleBot_ws/src
cd turtleBot_ws
catkin_make

#####################################

# clone our repository
cd src
git clone https://github.sydney.edu.au/dliu7477/MTRX3760-Project-1.git

# Package from others
## clone the turtleBot simulation if not
cd src
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ..
catkin_make

## clone the sensor_msg if not
cd src
git clone https://github.com/ros/common_msgs.git
cd common_msgs
git checkout jade_devel

###################################

# run the simulation
## go to correct directory
cd turtleBot_ws
source devel/setup.bash

## build the workspace
catkin_make

## choose buger as the model
export TURTLEBOT3_MODEL=burger

## Simulate an enclosed maze in Gazebo
roslaunch maze_package enclosed_world.launch

## Simulate an open track in Gazebo
roslaunch maze_package open_track.launch

## simulate in Rviz
roslaunch maze_package turtlebot3_gazebo_rviz.launch

## run the self-driving algorithm
roslaunch path_planning path_planning.launch
