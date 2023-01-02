#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

################################################################################

DRIVE_LIMIT = 1
STEER_LIMIT = 1
THROTTLE_CONST = 1/0.26
STEERING_CONST = -1/0.5236

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

pub_steering_command = rospy.Publisher('/autodrive/v1/steering_command', Float32, queue_size=1) # Testbed topic name was `steer_cmd`
pub_throttle_command = rospy.Publisher('/autodrive/v1/throttle_command', Float32, queue_size=1) # Testbed topic name was `drive_cmd`

def navCtrlCallback(data):
    global throttle, steering
    # Simulator
    throttle = boundDrive((THROTTLE_CONST * data.linear.x))
    steering = boundSteer(STEERING_CONST * data.angular.z)
    # Testbed
    '''
    throttle = boundDrive(0.2 + (THROTTLE_CONST * data.linear.x))
    steering = boundSteer(STEERING_CONST * data.angular.z)
    '''
    pub_throttle_command.publish(Float32(throttle))
    pub_steering_command.publish(Float32(steering))

################################################################################

if __name__=="__main__":

    rospy.init_node('nav_ctrl')
    rospy.Subscriber("cmd_vel", Twist, navCtrlCallback)
    rospy.spin()
