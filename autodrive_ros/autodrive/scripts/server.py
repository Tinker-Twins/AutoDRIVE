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
        # Actuator feedbacks
        throttle = float(data["Throttle"])
        steering_angle = float(data["Steering"])
        autodrive_ros_bridge.publish_actuator_feedbacks(throttle, steering_angle)
        # Wheel encoders
        encoder_angles = np.fromstring(data["Encoder Angles"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_encoder_data(encoder_angles)
        # IPS
        position = np.fromstring(data["Position"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_ips_data(position)
        # IMU
        orientation_quaternion = np.fromstring(data["Orientation Quaternion"], dtype=float, sep=' ')
        angular_velocity = np.fromstring(data["Angular Velocity"], dtype=float, sep=' ')
        linear_acceleration = np.fromstring(data["Linear Acceleration"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_imu_data(orientation_quaternion, angular_velocity, linear_acceleration)
        # Cooordinate transforms
        autodrive_ros_bridge.broadcast_transform("ego_vehicle", "map", position, orientation_quaternion)
        autodrive_ros_bridge.broadcast_transform("left_encoder", "ego_vehicle", np.asarray([-0.08, 0.0745, -0.015]), tf.transformations.quaternion_from_euler(0, encoder_angles[0]%6.283, 0))
        autodrive_ros_bridge.broadcast_transform("right_encoder", "ego_vehicle", np.asarray([-0.08, -0.0745, -0.015]), tf.transformations.quaternion_from_euler(0, encoder_angles[1]%6.283, 0))
        autodrive_ros_bridge.broadcast_transform("imu", "ego_vehicle", np.asarray([0, 0, 0]), np.asarray([0, 0, 0, 1]))
        autodrive_ros_bridge.broadcast_transform("lidar", "ego_vehicle", np.asarray([0.012, 0, 0.12475]), np.asarray([0, 0, -1, 0]))
        autodrive_ros_bridge.broadcast_transform("front_camera", "ego_vehicle", np.asarray([0.1034, 0, 0.06]), np.asarray([0, 0, 0, 1]))
        autodrive_ros_bridge.broadcast_transform("rear_camera", "ego_vehicle", np.asarray([-0.1034, 0, 0.06]), np.asarray([0, 0, -1, 0]))
        # LIDAR
        lidar_scan_rate = float(data["LIDAR Scan Rate"])
        lidar_range_array = np.fromstring(data["LIDAR Range Array"], dtype=float, sep=' ')
        lidar_intensity_array = np.fromstring(data["LIDAR Intensity Array"], dtype=float, sep=' ')
        autodrive_ros_bridge.publish_lidar_scan(lidar_scan_rate, lidar_range_array, lidar_intensity_array)
        # Cameras
        front_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["Front Camera Image"]))))
        rear_image = np.asarray(Image.open(BytesIO(base64.b64decode(data["Rear Camera Image"]))))
        autodrive_ros_bridge.publish_camera_images(front_image, rear_image)
        # Control commands
        sio.emit('Bridge', data={'Throttle': str(config.throttle_command), 'Steering': str(config.steering_command), 'Headlights': str(config.headlights_command), 'Indicators': str(config.indicators_command)})

if __name__ == '__main__':
    app = socketio.WSGIApp(sio) # Create socketio WSGI application
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever() # Deploy as an gevent WSGI server
