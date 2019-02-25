#!/bin/bash

echo $PWD

#Add the required environment variables
echo "export ROS_PACKAGE_PATH="$PWD":$ROS_PACKAGE_PATH" >> ~/.bashrc
echo ". "$PWD"/devel/setup.bash" >> ~/.bashrc

#Source the run commands file to update the terminal
source ~/.bashrc

#Make the workspace
catkin_make
