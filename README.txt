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


INSTALLATION
------------

1. Install ROS Kinetic and Gazebo.
2. Download the workspace to your machine.
3. Run the startup script "setup.bash" in the workspace.
4. Run "source ~/.bashrc" or restart the terminal.
5. Run the launch file with "roslaunch reactive_robot reactive_robot_$MAP.launch", replacing $MAP with either 
"hallway" or "room" depending on the map.

CONTROLLING THE ROBOT
---------------------

After following the instructions above, the program should be launched and driving around in either the hallway 
or the room world. 

Note: Gazebo frequently crashes upon startup, so if Gazebo doesn't open upon
running the launch file, you may need to kill the program (ctrl+c) and try
running it again.

The robot will begin driving around on its own unless prompted by keyboard commands from the user. The human can
control the robot by clicking on the console running roslaunch (to set focus) and then using the keys described 
in the turtlebot_teleop_keyboard. These keys are as follows:


Moving around
   u     i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

Note: The turtlebot_teleop_keyboard publishes a zero vector when pressing K. It also sends a zero vector when nothing 
is pressed. Therefore, the keyboard is not equipped to stop the robot with the K button. We recommend reducing speed 
with the other key pairs to stop the robot.


WORLDS
------

There are two environments provided for the robot to attempt to traverse, a hallway and a room. 

Excluding the structures inside the world, there are a few basic things that exist in both files. These are things 
that are not necessarily objects but do provide some foundational characteristics to the environment. The lighting, 
sky texture (commonly called the sky-box), and various physics attributes are all described in the world file, with 
things like physics tick speed, shadows, and light diffusion being defined here. The only model the worlds have in 
common is the ground plane. This is a square, centered at the origin, that extends out arbitrarily far compared to 
our actual models, and it has a defined friction as the robot travels over it.

Before we begin our outline of the models themselves, there are some common characteristics of the objects that we 
will define. Every object in the two models that has not already been described is a wall that is 0.15 meters thick, 
2.5 meters tall, immovable, solid, and resting on the ground plane. Also, all walls in these models connect at right 
angles (with one exception). Considering all of these factors, it is clear the the only difference between each wall 
will be it’s position and length.

The hallway consists of 3 individual walls. There are two walls (both 3m) that are parallel to each other. They are 
connected on one end by a wall (1.5m) that bridges the gap between.

The room is a more complicated model, containing 7 distinct walls (although there appear to only be 6). There are 3 
walls (all 4.5m) that form a “U” shape. Attached to the ends of the “U” are two different walls. The right side of 
the “U” (if viewing it from the bottom) has wall (3m) extending “inwards” towards the shape’s other side. The left 
side has a wall (1.5m) that continues upwards, effectively extending the wall it attaches to. This was done during 
model making to ensure everything was properly spaced. Extending from this wall is another wall (1.5m) that will 
extend towards the right side. In order to connect the top wall just described with the lower wall (3 meters) we 
will have a wall (1.5m)  that “drops down” and connects the two.


