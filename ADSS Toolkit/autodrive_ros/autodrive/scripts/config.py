#!/usr/bin/env python

# Import libraries
from attrdict import AttrDict

# ROS publishers and subscribers
pub_sub_dict = AttrDict({
    'subscribers': [
        {'topic':'/autodrive/ego_vehicle/throttle_command', 'type': 'float', 'name': 'sub_throttle_command'},
        {'topic':'/autodrive/ego_vehicle/steering_command', 'type': 'float', 'name': 'sub_steering_command'},
        {'topic':'/autodrive/ego_vehicle/headlights_command', 'type': 'int', 'name': 'sub_headlights_command'},
        {'topic':'/autodrive/ego_vehicle/indicators_command', 'type': 'int', 'name': 'sub_indicators_command'}
    ],
    'publishers': [
        {'topic': '/autodrive/ego_vehicle/throttle', 'type': 'float', 'name': 'pub_throttle'},
        {'topic': '/autodrive/ego_vehicle/steering_angle', 'type': 'float', 'name': 'pub_steering_angle'},
        {'topic': '/autodrive/ego_vehicle/left_encoder', 'type': 'joint_state', 'name': 'pub_left_encoder'},
        {'topic': '/autodrive/ego_vehicle/right_encoder', 'type': 'joint_state', 'name': 'pub_right_encoder'},
        {'topic': '/autodrive/ego_vehicle/ips', 'type': 'point', 'name': 'pub_ips'},
        {'topic': '/autodrive/ego_vehicle/imu', 'type': 'imu', 'name': 'pub_imu'},
        {'topic': '/autodrive/ego_vehicle/lidar', 'type': 'laser_scan', 'name': 'pub_lidar'},
        {'topic': '/autodrive/ego_vehicle/front_camera', 'type': 'image', 'name': 'pub_front_camera'},
        {'topic': '/autodrive/ego_vehicle/rear_camera', 'type': 'image', 'name': 'pub_rear_camera'}
    ]
})

# Vehicle control inputs
throttle_command = 0
steering_command = 0
headlights_command = 0
indicators_command = 0
