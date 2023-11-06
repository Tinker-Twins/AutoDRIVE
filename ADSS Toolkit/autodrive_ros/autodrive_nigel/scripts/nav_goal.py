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
from geometry_msgs.msg import PoseStamped

################################################################################

if __name__=="__main__":

    rospy.init_node('nav_goal')

    pub_nav_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    msg_nav_goal = PoseStamped()
    msg_nav_goal.header.stamp       = rospy.Time.now()
    msg_nav_goal.header.frame_id    = 'map'
    msg_nav_goal.pose.position.x    = rospy.get_param('/navigation_goal/pos_x')
    msg_nav_goal.pose.position.y    = rospy.get_param('/navigation_goal/pos_y')
    msg_nav_goal.pose.position.z    = rospy.get_param('/navigation_goal/pos_z')
    msg_nav_goal.pose.orientation.x = rospy.get_param('/navigation_goal/rot_x')
    msg_nav_goal.pose.orientation.y = rospy.get_param('/navigation_goal/rot_y')
    msg_nav_goal.pose.orientation.z = rospy.get_param('/navigation_goal/rot_z')
    msg_nav_goal.pose.orientation.w = rospy.get_param('/navigation_goal/rot_w')
    pub_nav_goal.publish(msg_nav_goal)

    rate = rospy.Rate(10) # Control frequency (Hz)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
