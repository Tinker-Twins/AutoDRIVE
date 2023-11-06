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
from attrdict import AttrDict

# ROS publishers and subscribers
pub_sub_dict = AttrDict({
    'subscribers': [
        # Vehicle data subscribers
        {'topic':'/autodrive/nigel_1/throttle_command', 'type': 'float', 'name': 'sub_throttle_command'},
        {'topic':'/autodrive/nigel_1/steering_command', 'type': 'float', 'name': 'sub_steering_command'},
        {'topic':'/autodrive/nigel_1/headlights_command', 'type': 'int', 'name': 'sub_headlights_command'},
        {'topic':'/autodrive/nigel_1/indicators_command', 'type': 'int', 'name': 'sub_indicators_command'},
        # Traffic light data subscribers
        {'topic':'/autodrive/signal_1/command', 'type': 'int', 'name': 'sub_signal_1_command'},
        {'topic':'/autodrive/signal_2/command', 'type': 'int', 'name': 'sub_signal_2_command'},
        {'topic':'/autodrive/signal_3/command', 'type': 'int', 'name': 'sub_signal_3_command'},
        {'topic':'/autodrive/signal_4/command', 'type': 'int', 'name': 'sub_signal_4_command'}
    ],
    'publishers': [
        # Vehicle data publishers
        {'topic': '/autodrive/nigel_1/throttle', 'type': 'float', 'name': 'pub_throttle'},
        {'topic': '/autodrive/nigel_1/steering', 'type': 'float', 'name': 'pub_steering'},
        {'topic': '/autodrive/nigel_1/left_encoder', 'type': 'joint_state', 'name': 'pub_left_encoder'},
        {'topic': '/autodrive/nigel_1/right_encoder', 'type': 'joint_state', 'name': 'pub_right_encoder'},
        {'topic': '/autodrive/nigel_1/ips', 'type': 'point', 'name': 'pub_ips'},
        {'topic': '/autodrive/nigel_1/imu', 'type': 'imu', 'name': 'pub_imu'},
        {'topic': '/autodrive/nigel_1/lidar', 'type': 'laser_scan', 'name': 'pub_lidar'},
        {'topic': '/autodrive/nigel_1/front_camera', 'type': 'image', 'name': 'pub_front_camera'},
        {'topic': '/autodrive/nigel_1/rear_camera', 'type': 'image', 'name': 'pub_rear_camera'},
        # Traffic light data publishers
        {'topic': '/autodrive/signal_1/state', 'type': 'int', 'name': 'pub_signal_1_state'},
        {'topic': '/autodrive/signal_2/state', 'type': 'int', 'name': 'pub_signal_2_state'},
        {'topic': '/autodrive/signal_3/state', 'type': 'int', 'name': 'pub_signal_3_state'},
        {'topic': '/autodrive/signal_4/state', 'type': 'int', 'name': 'pub_signal_4_state'}
    ]
})

# Vehicle control commands
throttle_command = 0 # [-1, 1]
steering_command = 0 # [-1, 1]
headlights_command = 0 # [0 = disabled, 1 = low beam, 2 = high beam]
indicators_command = 0 # [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]

# Traffic light control commands
signal_1_command = 0 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
signal_2_command = 0 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
signal_3_command = 0 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
signal_4_command = 0 # [0 = disabled, 1 = red, 2 = yellow, 3 = green]