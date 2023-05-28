#!/usr/bin/env python

# Import libraries
from attrdict import AttrDict

# ROS publishers and subscribers
pub_sub_dict = AttrDict({
    'subscribers': [
        # Vehicle data subscribers
        {'topic':'/autodrive/v1/throttle_command', 'type': 'float', 'name': 'sub_throttle_command'},
        {'topic':'/autodrive/v1/steering_command', 'type': 'float', 'name': 'sub_steering_command'},
        {'topic':'/autodrive/v1/headlights_command', 'type': 'int', 'name': 'sub_headlights_command'},
        {'topic':'/autodrive/v1/indicators_command', 'type': 'int', 'name': 'sub_indicators_command'},
        # Traffic light data subscribers
        {'topic':'/autodrive/tl1/command', 'type': 'int', 'name': 'sub_tl1_command'},
        {'topic':'/autodrive/tl2/command', 'type': 'int', 'name': 'sub_tl2_command'},
        {'topic':'/autodrive/tl3/command', 'type': 'int', 'name': 'sub_tl3_command'},
        {'topic':'/autodrive/tl4/command', 'type': 'int', 'name': 'sub_tl4_command'}
    ],
    'publishers': [
        # Vehicle data publishers
        {'topic': '/autodrive/v1/throttle', 'type': 'float', 'name': 'pub_throttle'},
        {'topic': '/autodrive/v1/steering', 'type': 'float', 'name': 'pub_steering'},
        {'topic': '/autodrive/v1/left_encoder', 'type': 'joint_state', 'name': 'pub_left_encoder'},
        {'topic': '/autodrive/v1/right_encoder', 'type': 'joint_state', 'name': 'pub_right_encoder'},
        {'topic': '/autodrive/v1/ips', 'type': 'point', 'name': 'pub_ips'},
        {'topic': '/autodrive/v1/imu', 'type': 'imu', 'name': 'pub_imu'},
        {'topic': '/autodrive/v1/lidar', 'type': 'laser_scan', 'name': 'pub_lidar'},
        {'topic': '/autodrive/v1/front_camera', 'type': 'image', 'name': 'pub_front_camera'},
        {'topic': '/autodrive/v1/rear_camera', 'type': 'image', 'name': 'pub_rear_camera'},
        # Traffic light data publishers
        {'topic': '/autodrive/tl1/state', 'type': 'int', 'name': 'pub_tl1_state'},
        {'topic': '/autodrive/tl2/state', 'type': 'int', 'name': 'pub_tl2_state'},
        {'topic': '/autodrive/tl3/state', 'type': 'int', 'name': 'pub_tl3_state'},
        {'topic': '/autodrive/tl4/state', 'type': 'int', 'name': 'pub_tl4_state'}
    ]
})

# Vehicle control commands
throttle_command = 0 # [-1, 1]
steering_command = 0 # [-1, 1]
headlights_command = 0 # [0 = disabled, 1 = low beam, 2 = high beam]
indicators_command = 0 # [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]

# Traffic light control commands
tl1_command = 0 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
tl2_command = 0 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
tl3_command = 0 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
tl4_command = 0 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
