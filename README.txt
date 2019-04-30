INTRODUCTION
------------
Current Maintainers: 	Justin Kleiber <jkleiber@ou.edu>
			Preston Gray <Preston.R.Gray-1@ou.edu>
			Trey Sullivent <trey.sullivent@ou.edu>

The workspace is for the Major Project for CS4023 at the University of Oklahoma.


GETTING UP AND RUNNING
-------------------------
1. Install ROS Kinetic and Gazebo.
2. Download the workspace to your machine.
3. Run the startup script "setup.bash" in the workspace.
4. Run "source ~/.bashrc" or restart the terminal.
5. Run the launch file with "roslaunch reactive_robot reactive_robot.launch".
6. SSH into the TurtleBot computer, then run “roslaunch turtlebot_bringup minimal.launch --screen” and “roslaunch turtlebot_bringup 3dsensor.launch” in separate terminals.
7. On the master computer, run "roslaunch yeetbotics yeetbotics.launch" and "rosrun yeet_board keyboard_node.py"

DRIVING THE ROBOT
----------------------
$x: The x coordinate of the specified node
$y: The y coordinate of the specified node
The command input can be used to populate nodes in the map by typing "node $x $y -p".
The Turtlebot can be sent to a specific node with "goto $x $y" or "node $x $y -g"
The keyboard node can enter manual rescue mode by typing "raw" and using the following commands: (WARNING: Raw mode may disrupt odometry and decrease localization accuracy.

Direction Control
        w    
   a    s    d

Speed Control
q/e : increase/decrease max speeds by 10%
