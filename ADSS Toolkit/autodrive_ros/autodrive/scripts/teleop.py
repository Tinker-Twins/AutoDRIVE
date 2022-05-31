#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

################################################################################

DRIVE_LIMIT = 1
STEER_LIMIT = 1
DRIVE_STEP_SIZE = 0.2
STEER_STEP_SIZE = 0.2

info = """
-------------------------------------
AutoDRIVE - Nigel Teleoperation Panel
-------------------------------------

             Q   W   E
             A   S   D
                 X

W/S : Increase/decrease drive command
D/A : Increase/decrease steer command
Q   : Zero steer
E   : Emergency brake
X   : Force stop and reset

Press CTRL+C to quit

NOTE: Press keys within this terminal
-------------------------------------
"""

error = """
ERROR: Communication failed!
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def boundSteer(steer_cmd):
    steer_cmd = constrain(steer_cmd, -STEER_LIMIT, STEER_LIMIT)
    return steer_cmd

def boundDrive(drive_cmd):
    drive_cmd = constrain(drive_cmd, -DRIVE_LIMIT, DRIVE_LIMIT)
    return drive_cmd

################################################################################

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('testbed_teleop')
    steer_pub = rospy.Publisher('steer_cmd', Float32, queue_size=10)
    drive_pub = rospy.Publisher('drive_cmd', Float32, queue_size=10)

    throttle = 0.0
    steering = 0.0

    try:
        print(info)

        while(1):
            key = getKey()
            if key == 'w' :
                throttle = boundDrive(throttle + DRIVE_STEP_SIZE)
            elif key == 's' :
                throttle = boundDrive(throttle - DRIVE_STEP_SIZE)
            elif key == 'a' :
                steering = boundSteer(steering - STEER_STEP_SIZE)
            elif key == 'd' :
                steering = boundSteer(steering + STEER_STEP_SIZE)
            elif key == 'q' :
                steering = 0.0
            elif key == 'e' :
                throttle = 0.0
            elif key == 'x' :
                throttle = 0.0
                steering = 0.0
            else:
                if (key == '\x03'): # CTRL+C
                    break

            drive_pub.publish(throttle)
            steer_pub.publish(steering)

    except:
        print(error)

    finally:
        throttle = 0.0
        steering = 0.0
        drive_pub.publish(throttle)
        steer_pub.publish(steering)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
