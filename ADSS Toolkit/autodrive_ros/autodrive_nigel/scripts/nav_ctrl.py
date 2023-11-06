#!/usr/bin/env python

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

################################################################################

DRIVE_LIMIT = 1
STEER_LIMIT = 1
THROTTLE_CONST = 1/0.26
STEERING_CONST = 1/0.5236

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

pub_steering_command = rospy.Publisher('/autodrive/nigel_1/steering_command', Float32, queue_size=1) # Testbed topic name was `steer_cmd`
pub_throttle_command = rospy.Publisher('/autodrive/nigel_1/throttle_command', Float32, queue_size=1) # Testbed topic name was `drive_cmd`

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
