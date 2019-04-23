#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('yeet_board')
import rospy

from geometry_msgs.msg import Twist
from yeet_msgs.msg import move

import sys, select, termios, tty

class color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'


instructions = """
\n\n\n\n\n\n\n\n
Moving around:
         ^
         |
         w     
<-- a    s    d -->
         |
         v

q : speed up
e : slow down 
anything else : stop

CTRL-C (^C) to enter token mode
CTRL-Z (^Z) to quit
"""

# 'key': (movement direction, turn direction, move speed modify)
keyBindings = {
        'w':( 1, 0, 0),
        's':(-1, 0, 0),
        'a':( 0,-1, 0),
        'd':( 0, 1, 0),
        'q':( 0, 0,-1),
        'e':( 0, 0, 1),
        ' ':( 0, 0, 0)
    }

exitCommands = [
    'exit',
    'quit',
    'q',
]

stdCommands = [
    'goto',
    'raw',
    'help',
]

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def status_report(drive, turn, drive_speed, turn_speed, key):
    return "drive:%s\tturn:%s\ndrive speed:%s\tturn speed:%s\tchar:%s" % (float(drive), float(turn), float(drive_speed), float(turn_speed), key)

def help():
    print("Current commands are",)
    for command in stdCommands:
        print(command, '\t')
    print()


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

                                                                #TODO: Message type
    d_star_pub = rospy.Publisher('yeet_planning/planning_update', move, queue_size = 10)
    keyboard_pub = rospy.Publisher('yeet_mergency/keyboard', move, queue_size = 10)
    rospy.init_node('keyboard_node')
    
    drive = 0   # drive command to be published in the move (-/+:backwards/forwards)
    turn = 0    # turn speed (-/+:TODO/TODO)
    drive_speed = 0.5   # drive speed scaler
    turn_speed = 0.5    # turn speed scaler (const)

    mode = "token"

    key = ' '

    try:
        while(1):

            if mode == "token":
                color.BOLD + 'Enter command: ' + color.END
                print(color.BOLD + 'Enter command: ' + color.END, end='')
                s = raw_input()
                tokens = s.split()
                
                if tokens and tokens[0] is not '':
                    
                    if tokens[0] in exitCommands:
                        break
                    
                    elif tokens[0] == "goto":
                        try:
                            x_coord = int(tokens[1])
                            y_coord = int(tokens[2])
                            print("Going to coordinate x=", x_coord, ", y=", y_coord)
                            # TODO: Publish a message to D*
                            # TODO: Change to node message (TODO: make node message)
                            node_msg = move()
                            node_msg.drive = drive
                            node_msg.turn = turn
                            d_star_pub.publish(move_msg)

                        except IndexError:
                            print(color.RED + "goto requires ", color.BOLD + "two" + color.END,  color.RED + " integer parameters:"  + color.END, "Usage \' goto 4 5 \'")
                        except ValueError:
                            print(color.RED + "goto requires two ", color.BOLD + "integer" + color.END,  color.RED + " parameters:"  + color.END, "Usage \' goto 4 5 \'")


                    elif tokens[0] == "raw":
                        print("* Switched to raw mode")
                        mode = 'raw'
                        print(instructions)

                    elif tokens[0] == 'help':
                        help()
                    else:
                        print("* Oops!", tokens[0], "is not a command.")
                        help()
                print()



            else: # mode == "raw"
                print(instructions)
                print(status_report(drive, turn, drive_speed, turn_speed, key))

                key = getKey()

                if key in keyBindings.keys():
                    drive = keyBindings[key][0] * drive_speed
                    turn = keyBindings[key][1] * turn_speed
                    drive_speed += keyBindings[key][2] * 0.1
                
                else:
                    drive = 0
                    turn = 0                    
                    if (key == '\x03'):         # Ctrl-C
                        print("Switched to token mode")
                        mode = "token"
                        key = ' '   # reset key to avoid ugly print

                    elif (key == '\x1A'):       # Ctrl-Z
                        break
                        
                
            move_msg = move()
            move_msg.drive = drive
            move_msg.turn = turn
            keyboard_pub.publish(move_msg)


    except Exception as e:
        print(e)

    finally:
        move_msg = move()
        move_msg.drive = drive
        move_msg.turn = turn
        keyboard_pub.publish(move_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
