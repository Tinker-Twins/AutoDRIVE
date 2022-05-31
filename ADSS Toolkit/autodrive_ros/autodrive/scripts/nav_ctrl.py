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

steer_pub = rospy.Publisher('steer_cmd', Float32, queue_size=10)
drive_pub = rospy.Publisher('drive_cmd', Float32, queue_size=10)

def navCtrlCallback(data):
    global throttle, steering
    throttle = boundDrive(0.2 + (THROTTLE_CONST * data.linear.x))
    steering = boundSteer(STEERING_CONST * data.angular.z)
    drive_pub.publish(Float32(throttle))
    steer_pub.publish(Float32(steering))

################################################################################

if __name__=="__main__":

    rospy.init_node('nav_ctrl')
    rospy.Subscriber("cmd_vel", Twist, navCtrlCallback)
    rospy.spin()
