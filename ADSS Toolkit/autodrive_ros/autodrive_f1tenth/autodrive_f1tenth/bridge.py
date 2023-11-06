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

# Import libraries
import rospy
import tf
from std_msgs.msg import Int32, Float32, Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState, Imu, LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import autodrive_f1tenth.config as config

msg_types = {
    'int': Int32,
    'float': Float32,
    'joint_state': JointState,
    'point': Point,
    'imu': Imu,
    'laser_scan': LaserScan,
    'image':Image
}

class Bridge(object):
    def __init__(self, pub_sub_dict):
        rospy.init_node('autodrive_ros_bridge') # Initialize node
        self.cv_bridge = CvBridge()
        self.callbacks = {
            # Vehicle data subscriber callbacks
            '/autodrive/f1tenth_1/throttle_command': self.callback_throttle_command,
            '/autodrive/f1tenth_1/steering_command': self.callback_steering_command,
        }
        # Subscribers
        self.subscribers = [rospy.Subscriber(e.topic, msg_types[e.type], self.callbacks[e.topic])
                            for e in pub_sub_dict.subscribers]
        # Publishers
        self.publishers = {e.name: rospy.Publisher(e.topic, msg_types[e.type], queue_size=1)
                           for e in pub_sub_dict.publishers}

    #########################################################
    # ROS MESSAGE GENERATING FUNCTIONS
    #########################################################

    def create_int_msg(self, val):
        i = Int32()
        i.data = val
        return i

    def create_float_msg(self, val):
        f = Float32()
        f.data = val
        return f

    def create_joint_state_msg(self, joint_angle, joint_name, frame_id):
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = frame_id
        js.name = [joint_name]
        js.position = [joint_angle]
        js.velocity = []
        js.effort = []
        return js

    def create_point_msg(self, position):
        p = Point()
        p.x = position[0]
        p.y = position[1]
        p.z = position[2]
        return p

    def create_imu_msg(self, orientation_quaternion, angular_velocity, linear_acceleration):
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'imu'
        imu.orientation.x = orientation_quaternion[0]
        imu.orientation.y = orientation_quaternion[1]
        imu.orientation.z = orientation_quaternion[2]
        imu.orientation.w = orientation_quaternion[3]
        imu.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        imu.angular_velocity.x = angular_velocity[0]
        imu.angular_velocity.y = angular_velocity[1]
        imu.angular_velocity.z = angular_velocity[2]
        imu.angular_velocity_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        imu.linear_acceleration.x = linear_acceleration[0]
        imu.linear_acceleration.y = linear_acceleration[1]
        imu.linear_acceleration.z = linear_acceleration[2]
        imu.linear_acceleration_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        return imu

    def create_laser_scan_msg(self, lidar_scan_rate, lidar_range_array, lidar_intensity_array):
        ls = LaserScan()
        ls.header = Header()
        ls.header.stamp = rospy.Time.now()
        ls.header.frame_id = 'lidar'
        ls.angle_min = -2.35619 # Minimum angle of laser scan (0 degrees)
        ls.angle_max = 2.35619 # Maximum angle of laser scan (270 degrees)
        ls.angle_increment = 0.004363323 # Angular resolution of laser scan (0.25 degree)
        ls.time_increment = (1 / lidar_scan_rate) / 360 # Time required to scan 1 degree
        ls.scan_time = ls.time_increment * 360 # Time required to complete a scan of 360 degrees
        ls.range_min = 0.06 # Minimum sensor range (in meters)
        ls.range_max = 10.0 # Maximum sensor range (in meters)
        ls.ranges = lidar_range_array
        ls.intensities = lidar_intensity_array
        return ls

    def create_image_msg(self, image_array, frame_id):
        img = self.cv_bridge.cv2_to_imgmsg(image_array, encoding="rgb8")
        img.header = Header()
        img.header.stamp = rospy.Time.now()
        img.header.frame_id = frame_id
        return img

    def broadcast_transform(self, child_frame_id, parent_frame_id, position_tf, orientation_tf):
        tb = tf.TransformBroadcaster()
        tb.sendTransform(position_tf, orientation_tf, rospy.Time.now(), child_frame_id, parent_frame_id)

    #########################################################
    # ROS PUBLISHER FUNCTIONS
    #########################################################

    def publish_actuator_feedbacks(self, throttle, steering):
        self.publishers['pub_throttle'].publish(self.create_float_msg(throttle))
        self.publishers['pub_steering'].publish(self.create_float_msg(steering))

    def publish_encoder_data(self, encoder_angles):
        self.publishers['pub_left_encoder'].publish(self.create_joint_state_msg(encoder_angles[0], "left_encoder", "left_encoder"))
        self.publishers['pub_right_encoder'].publish(self.create_joint_state_msg(encoder_angles[1], "right_encoder", "right_encoder"))

    def publish_ips_data(self, position):
        self.publishers['pub_ips'].publish(self.create_point_msg(position))

    def publish_imu_data(self, orientation_quaternion, angular_velocity, linear_acceleration):
        self.publishers['pub_imu'].publish(self.create_imu_msg(orientation_quaternion, angular_velocity, linear_acceleration))

    def publish_lidar_scan(self, lidar_scan_rate, lidar_range_array, lidar_intensity_array):
        self.publishers['pub_lidar'].publish(self.create_laser_scan_msg(lidar_scan_rate, lidar_range_array, lidar_intensity_array))

    def publish_camera_images(self, front_camera_image):
        self.publishers['pub_front_camera'].publish(self.create_image_msg(front_camera_image, "front_camera"))

    #########################################################
    # ROS SUBSCRIBER CALLBACKS
    #########################################################

    def callback_throttle_command(self, throttle_command):
        config.throttle_command = np.round(throttle_command.data, 3)

    def callback_steering_command(self, steering_command):
        config.steering_command = np.round(steering_command.data, 3)