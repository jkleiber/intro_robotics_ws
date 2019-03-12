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


INSTALLATION INSTRUCTIONS
-------------------------
1. Install ROS Kinetic and Gazebo.
2. Download the workspace to your machine.
3. Run the startup script "setup.bash" in the workspace.
4. Run "source ~/.bashrc" or restart the terminal.
5. Run the launch file with "roslaunch reactive_robot reactive_robot.launch".
6. SSH into the TurtleBot computer, then run “roslaunch turtlebot_bringup minimal.launch --screen” and “roslaunch turtlebot_bringup 3dsensor.launch” in separate terminals.

OPERATION INSTRUCTIONS
----------------------
The robot will begin driving around on its own unless prompted by keyboard commands from the user. The human can control the robot by clicking on the console running roslaunch (to set focus) and then using the keys described in the turtlebot_teleop_keyboard. These keys are as follows:
Direction Control
   u    i    o
   j    k    l
   m    ,    .
Speed Control
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%


Note: The turtlebot_teleop_keyboard publishes a zero vector when pressing K. It also sends a zero vector when nothing is pressed. Therefore, the keyboard is not equipped to stop the robot with the K button. We recommend reducing speed with the other key pairs to stop the robot.

The map from the most recent run will be saved in ~/reactive_robot_map.yaml. 
The program can be ended by pressing ctrl+c twice.

LAUNCH FILES
------------
The launch files are simple and contain only a couple primary parts. In the first part, we launch our nodes. Then we launch the keyboard and remap its outputs to suit our needs. Finally we launch gmapping to get mapping capabilities.


CODE DESCRIPTION
----------------

Overview:
We implemented a reactive paradigm using our interpretation of schema theory, shown in the 
figure above. We chose this architecture because it had a logical flow from sensor input to 
motor output, and due to its modular design. By implementing a central decision node based 
on the priorities listed in the assignment, we were able to control which releaser the 
program needed to activate. This made the design simple and modular, as releasers and new 
features can be easily added into the architecture. Each node is specifically focused on 
its own area of expertise, which is essential in the schema architecture. Due to the nodes 
reacting to their own stimuli from the environment, there is less deliberation to make in 
the decision node. Additionally, the code is made more robust due to sensor integration 
inherent to our design - explained further in the obstacle node and autodrive nodes below. 
These components, along with other different parts of our architecture are described below.

Autodrive Node:
The autodrive node subscribes to the odometry topic in order to ascertain the current position
 and angle of the robot. It keeps track of the current position and angle, while also calculating 
 the distance from the last time the robot turned. After the robot has travelled a foot, the 
 robot calculates a random angle change between -15 and 15 degrees, and then executes an in-place 
 turn. This node uses an instance of the Drivetrain class (discussed later) to calculate a 
 geometry_msgs/Twist message to output to the decision node.

Collision Node:
The collision node subscribes to the turtlebot’s bumper topic. Upon either a bumper press or 
release, this node will update its internal state of the bumpers in order to know if we need 
to halt or not. The code utilizes a struct to contain the pressed status of the left, right 
and center bumpers, which is updated in the bumper topic’s callback function. Each time this 
callback is run, the node publishes a boolean value to the collision topic for the decision 
node to determine if the robot has encountered a collision.

Obstacle Node:
The obstacle node subscribes to the scan topic in order to parse out information about 
obstacles in the environment. Obstacle detection and retention was limited to obstacles 
within two feet of the scanner. Given the position of the scanner on the turtlebot, this 
resulted in detection of obstacles roughly one foot away. We deduced that in the current 
environments that the turtlebot is navigating, any objects that the turtlebot detects 
outside of a narrow range directly in front of the scanner (defined as 75 scanner samples) 
will naturally be asymmetric due to the nature of perception, resulting in any objects inside 
of that narrow range defaulting to symmetric. This simplifies the overall complexity of our 
obstacle state deductions significantly. If two asymmetric objects were detected, the distance 
of both objects was found and a Twist message was updated to turn away from the closer one.

Decision Node:
The central decision node subscribes to all other nodes, as well as the keyboard inputs and 
odometry topic and contains a multitude of callbacks in order to update the state of the robot. 
It then deduces the cascading priority list of reactions based on the state as well as continually 
saving the map every five seconds. In addition to making reactive decisions, the decision node also 
is used to implement fixed action patterns. We have implemented two of such patterns, one to halt 
the robot after collisions and another to make the robot turn 180 degrees during an escape sequence. 
In the case of collisions, the robot will halt until keyboard input is registered to extract the 
robot from the collision scenario.

Drivetrain:
The drivetrain class is a helper class used by some of the nodes to calculate and store turtlebot 
output in a more manipulable format. The drivetrain includes simple functions for setting turn 
speed and forward speed. In addition to these functions, the drivetrain uses a P controller to 
turn the robot to a target angle. P stands for proportional and corrects the robot’s output based 
on the current angle error. Finally, the drivetrain class contains functionality to do angle 
arithmetic, so the robot always knows which way to turn given a current angle and target angle.
