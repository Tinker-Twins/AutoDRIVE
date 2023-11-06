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
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
import socketio
import numpy as np
import base64
from io import BytesIO
from PIL import Image
from tf.transformations import quaternion_from_euler
import autodrive_f1tenth.config as config
from autodrive_f1tenth.bridge import Bridge

# AutoDRIVE-ROS Bridge instance
autodrive_ros_bridge = Bridge(config.pub_sub_dict)

# Initialize the server
sio = socketio.Server(async_mode='gevent')

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print("Connected!")

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    if data:
        ########################################################################
        # VEHICLE DATA
        ########################################################################
        # Actuator feedbacks
        throttle = float(data["V1 Throttle"])
        steering = float(data["V1 Steering"])
        autodrive_ros_bridge.publish_actuator_feedbacks(throttle, steering)
        # Wheel encoders
        encoder_angles = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_encoder_data(encoder_angles)
        # IPS
        position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_ips_data(position)
        # IMU
        orientation_quaternion = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
        angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
        linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration)
        # Cooordinate transforms
        autodrive_ros_bridge.broadcast_transform("f1tenth_1", "map", position, orientation_quaternion) # Vehicle frame defined at center of rear axle
        autodrive_ros_bridge.broadcast_transform("left_encoder", "f1tenth_1", np.asarray([0.0, 0.12, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[0]%6.283, 0.0))
        autodrive_ros_bridge.broadcast_transform("right_encoder", "f1tenth_1", np.asarray([0.0, -0.12, 0.0]), quaternion_from_euler(0.0, 120*encoder_angles[1]%6.283, 0.0))
        autodrive_ros_bridge.broadcast_transform("ips", "f1tenth_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        autodrive_ros_bridge.broadcast_transform("imu", "f1tenth_1", np.asarray([0.08, 0.0, 0.055]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        autodrive_ros_bridge.broadcast_transform("lidar", "f1tenth_1", np.asarray([0.2733, 0.0, 0.096]), np.asarray([0.0, 0.0, 0.0, 1.0]))
        autodrive_ros_bridge.broadcast_transform("front_camera", "f1tenth_1", np.asarray([-0.015, 0.0, 0.15]), np.asarray([0, 0.0871557, 0, 0.9961947]))
        autodrive_ros_bridge.broadcast_transform("front_left_wheel", "f1tenth_1", np.asarray([0.33, 0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537-2*0.0765*np.tan(steering)))))
        autodrive_ros_bridge.broadcast_transform("front_right_wheel", "f1tenth_1", np.asarray([0.33, -0.118, 0.0]), quaternion_from_euler(0.0, 0.0, np.arctan((2*0.141537*np.tan(steering))/(2*0.141537+2*0.0765*np.tan(steering)))))
        autodrive_ros_bridge.broadcast_transform("rear_left_wheel", "f1tenth_1", np.asarray([0.0, 0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles[0]%6.283, 0.0))
        autodrive_ros_bridge.broadcast_transform("rear_right_wheel", "f1tenth_1", np.asarray([0.0, -0.118, 0.0]), quaternion_from_euler(0.0, encoder_angles[1]%6.283, 0.0))
        # LIDAR
        lidar_scan_rate = float(data["V1 LIDAR Scan Rate"])
        lidar_range_array = np.fromstring(data["V1 LIDAR Range Array"], dtype=float, sep=' ')
        lidar_intensity_array = np.fromstring(data["V1 LIDAR Intensity Array"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array)
        # Camera
        front_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Front Camera Image"]))))
        autodrive_ros_bridge.publish_camera_images(front_camera_image)

        ########################################################################
        # CONTROL COMMANDS
        ########################################################################
        # Vehicle control commands
        sio.emit('Bridge', data={'V1 Throttle': str(config.throttle_command), 'V1 Steering': str(config.steering_command)})

if __name__ == '__main__':
    app = socketio.WSGIApp(sio) # Create socketio WSGI application
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever() # Deploy as an gevent WSGI server