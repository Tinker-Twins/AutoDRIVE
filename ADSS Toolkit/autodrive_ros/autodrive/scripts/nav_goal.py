#!/usr/bin/env python

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
