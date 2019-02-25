#!/bin/bash

#Pretend to be interactive
set -i $-

echo "Workspace directory:"$PWD;

echo "Removing pre-built code"
rm -rf build

#Add ROS Kinetic to the bashrc
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc;
set -i $-
source ~/.bashrc;

#Add the required environment variables
echo "Adding the workspace to the ROS_PACKAGE_PATH, and the .bashrc";
echo "export ROS_PACKAGE_PATH="$PWD":$ROS_PACKAGE_PATH" >> ~/.bashrc;

echo ". "$PWD"/devel/setup.bash";
echo ". "$PWD"/devel/setup.bash" >> ~/.bashrc;

echo "source "$PWD"/devel/setup.bash";
echo "source "$PWD"/devel/setup.bash" >> ~/.bashrc;


#Source the run commands file to update the terminal
set -i $-
source ~/.bashrc;

#Make the workspace
catkin_make;

source ~/.bashrc;
