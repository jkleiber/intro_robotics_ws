#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

//ROS libs and msgs
#include <ros/ros.h>

//User libs and msgs
#include <yeet_msgs/move.h>


//Publisher
ros::Publisher keyboard_pub;
yeet_msgs::move move_msg;   //Published message


/*
// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
    {'w', {1, 0, 0, 0}},
    {'s', {1, 0, 0, -1}},

  
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
  
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Reminder message

const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
)";
*/

// Init variables
float drive_speed(0.5); // Linear velocity (m/s)
float turn_speed(1.0); // Angular velocity (rad/s)
float drive = 0;
float turn = 0;

char key = ' ';

// For non-blocking keyboard inputs
//Credit: https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

/**
 * Runs the loop needed to handle keyboard functions
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "keyboard_node");

    //Set up the node handle for collision detection
    ros::NodeHandle keyboard_node;
    
    //TODO: Choose topic name between keyboard_node and human_control_node
    //Publish state to the collision topic
    keyboard_pub = keyboard_node.advertise<yeet_msgs::move>("/yeet_board/TODO", 10);

    //Set the loop rate of the decision function to 100 Hz
    ros::Rate loop_rate(100);


    //Make a decisision for what to do
    while(ros::ok())
    {
        //Perform all the callbacks
        ros::spinOnce();
        
        key = getch();

        /*
        // If the key corresponds to a key in moveBindings
        if (moveBindings.count(key) == 1)
        {
            // Grab the direction data
            x = moveBindings[key][0];
            y = moveBindings[key][1];
            z = moveBindings[key][2];
            th = moveBindings[key][3];

            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
        }

        // Otherwise if it corresponds to a key in speedBindings
        else if (speedBindings.count(key) == 1)
        {
            // Grab the speed data
            speed = speed * speedBindings[key][0];
            turn = turn * speedBindings[key][1];

            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
        }

        // Otherwise, set the robot to stop
        else
        {
            drive = 0;
            turn = 0;


            // If ctrl-C (^C) was pressed, terminate the program
            if (key == '\x03')
            {
                printf("Exiting...\n");
                break;
            }

            printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
        }

        */

        // TODO: Update the move_msg


        // Publish it and resolve any remaining callbacks
        keyboard_pub.publish(move_msg);

        //Finish the current loop
        loop_rate.sleep();
    }

    return 0;
}