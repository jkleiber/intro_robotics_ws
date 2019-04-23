#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('yeet_board')
import rospy

from geometry_msgs.msg import Twist
from yeet_msgs.msg import move

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
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
"""

'''
moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }
'''
# 'key': (movement direction, turn direction, move speed modify)
keyBindings = {
        'w':( 1, 0, 0),
        's':(-1, 0, 0),
        'a':( 0,-1, 0),
        'd':( 0, 1, 0),
        'q':( 0, 0, 1),
        'e':( 0, 0,-1)
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('yeet_mergency/keyboard', move, queue_size = 10)
    rospy.init_node('keyboard_node')

    #speed = rospy.get_param("~speed", 0.5)
    #turn = rospy.get_param("~turn", 1.0)
    
    drive = 0
    turn = 0
    
    drive_speed = 0.5
    turn_speed = 0.5

    status = 0

    mode = 0

    x_coord = 0
    y_coord = 0

    try:
        while(1):

            if mode == 1:
                s = raw_input()
                tokens = s.split()
                
                if tokens[0] == "goto":
                    x_coord = int(tokens[1])
                    y_coord = int(tokens[2])
                    print("Going to coordinate x=", x_coord, ", y=", y_coord)
                
                elif tokens[0] == "raw":
                    print("Switched to raw mode")
                    mode = 0
                    print(msg)
                
                elif tokens[0] == "exit":
                    break
                

            else:
                print(msg)
                print(vels(speed,turn))
                key = getKey()

                if key in keyBindings.keys():
                    drive = moveBindings[key][0] * drive_speed
                    turn = moveBindings[key][1] * turn_speed
                    drive_speed += moveBindings[key][2] * 0.1
                
                elif False:
                #elif key in speedBindings.keys():
                    speed = speed * speedBindings[key][0]
                    turn = turn * speedBindings[key][1]

                    print(vels(speed,turn))
                    if (status == 14):
                        print(msg)
                    status = (status + 1) % 15
                
                else:
                    drive = 0
                    turn = 0
                    drive_speed += moveBindings[key][2] * 0.1
                    
                    # Detect Ctrl-C combo
                    if (key == '\x03'):
                        print("Switched to token mode")
                        mode = 1
                        
                
            move_msg = move()
            move_msg.linear.x = 0; move_msg.linear.y = 0; move_msg.linear.z = 0
            move_msg.angular.x = 0; move_msg.angular.y = 0; move_msg.angular.z = 0
            pub.publish(move_msg)

    except Exception as e:
        print(e)

    finally:
        move_msg = move()
        move_msg.linear.x = 0; move_msg.linear.y = 0; move_msg.linear.z = 0
        move_msg.angular.x = 0; move_msg.angular.y = 0; move_msg.angular.z = 0
        pub.publish(move_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
