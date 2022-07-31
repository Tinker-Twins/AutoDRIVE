#!/usr/bin/env python

# Import libraries
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
import socketio
import numpy as np
import base64
from io import BytesIO
from PIL import Image
import tf
import config
from bridge import Bridge

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
        autodrive_ros_bridge.broadcast_transform("v1", "map", position, orientation_quaternion)
        autodrive_ros_bridge.broadcast_transform("left_encoder", "v1", np.asarray([-0.08, 0.0745, -0.015]), tf.transformations.quaternion_from_euler(0, encoder_angles[0]%6.283, 0))
        autodrive_ros_bridge.broadcast_transform("right_encoder", "v1", np.asarray([-0.08, -0.0745, -0.015]), tf.transformations.quaternion_from_euler(0, encoder_angles[1]%6.283, 0))
        autodrive_ros_bridge.broadcast_transform("imu", "v1", np.asarray([0, 0, 0]), np.asarray([0, 0, 0, 1]))
        autodrive_ros_bridge.broadcast_transform("lidar", "v1", np.asarray([0.012, 0, 0.12475]), np.asarray([0, 0, -1, 0]))
        autodrive_ros_bridge.broadcast_transform("front_camera", "v1", np.asarray([0.1034, 0, 0.06]), np.asarray([0, 0, 0, 1]))
        autodrive_ros_bridge.broadcast_transform("rear_camera", "v1", np.asarray([-0.1034, 0, 0.06]), np.asarray([0, 0, -1, 0]))
        # LIDAR
        lidar_scan_rate = float(data["V1 LIDAR Scan Rate"])
        lidar_range_array = np.fromstring(data["V1 LIDAR Range Array"], dtype=float, sep=' ')
        lidar_intensity_array = np.fromstring(data["V1 LIDAR Intensity Array"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array)
        # Cameras
        front_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Front Camera Image"]))))
        rear_camera_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Rear Camera Image"]))))
        autodrive_ros_bridge.publish_camera_images(front_camera_image, rear_camera_image)

        ########################################################################
        # TRAFFIC LIGHT DATA
        ########################################################################
        # Traffic light states
        tl1_state = int(data["TL1 State"])
        tl2_state = int(data["TL2 State"])
        tl3_state = int(data["TL3 State"])
        tl4_state = int(data["TL4 State"])
        autodrive_ros_bridge.publish_tl_states(tl1_state, tl2_state, tl3_state, tl4_state)

        ########################################################################
        # CONTROL COMMANDS
        ########################################################################
        # Vehicle and traffic light control commands
        sio.emit('Bridge', data={'V1 Throttle': str(config.throttle_command), 'V1 Steering': str(config.steering_command), 'V1 Headlights': str(config.headlights_command), 'V1 Indicators': str(config.indicators_command),
                                 'TL1 State': str(config.tl1_command), 'TL2 State': str(config.tl2_command), 'TL3 State': str(config.tl3_command), 'TL4 State': str(config.tl4_command)})

if __name__ == '__main__':
    app = socketio.WSGIApp(sio) # Create socketio WSGI application
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever() # Deploy as an gevent WSGI server
