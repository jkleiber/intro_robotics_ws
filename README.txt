INTRODUCTION
------------

Current Maintainers: 	Justin Kleiber <jkleiber@ou.edu>
						Preston Gray <Preston.R.Gray-1@ou.edu>
						Trey Sullivent <trey.sullivent@ou.edu>

The workspace is for the "Reactive Robotics using ROS and Gazebo" project for 
CS4023 at the University of Oklahoma. This robot is capable of several key
functions that will be described later. 

This project has been written is C++ and is not currently designed for expansion.
Some of the code has been written to allow us to modify it later, but the workspace
as it currently exists is primarily for launching and operating the robot for this
specific project.



GETTING STARTED
---------------

To start, the computer where the robot simulation will be run needs to be 
equipped with all the prerequisites. These include ROS Kinetic and Gazebo,
which can be installed by following the links here: 
http://wiki.ros.org/kinetic/Installation and http://gazebosim.org/.

The installation of the robot code itself is only two steps. The workspace 
must be imported onto the computer that is running hte simulation, and the
nodes must be built.

Once you have downloaded the workspace, simply run the command "catkin_make"
from a terminal within the workspace.

After it has build sucessfully, we will run the launch file. This can be done 
with the command "roslaunch reactive_robot reactive_robot.launch".

Now Gazebo should open and the bot will begin to drive around. 

Note: Gazebo frequently crashes upon startup, so if Gazebo doesn't open upon 
running the launch file, you may need to kill the program (ctrl+c) and try 
running it again.


Summary:
1. Install ROS Kinetic and Gazebo.
2. Download the workspace to your machine.
3. Run the startup script "setup.bash" in the workspace.
4. Run "source ~/.bashrc" or restart the terminal.
5. Run the launch file with "roslaunch reactive_robot reactive_robot_$MAP.launch", replacing $MAP with either "hallway" or "room" depending on the map.


WORLDS
------

//TODO: Determine if we need different maps for hallway and room or just 1.

Room:

The room begins by placing a floor plane on the ground. This is what the robot will 
drive upon and is 100m by 100m, centered at the origin. Around the origin, we have
built a room. The room is a modified square, shaped somewhat like the US state Utah.

The room is made of 7 distinct walls (Visually it appears that there are only 6). 
These walls are all of thickness 15cm and height 1.25m. There is a wall 




DESIGN DECISIONS
----------------


