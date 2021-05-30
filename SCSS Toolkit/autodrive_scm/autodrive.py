#!/usr/bin/env python

# Import libraries
import sqlite3
import socketio
import eventlet
from flask import Flask
import numpy as np
import base64
from io import BytesIO
from PIL import Image
import cv2

# Receive vehicle sensor data [function]
def receive_vehicle_sensor_data(data, verbose=False):
    global steering_angle, throttle, front_image, rear_image, encoder_ticks, encoder_angles, position, orientation_quaternion, orientation_euler_angles, angular_velocity, linear_acceleration, lidar_range_array, lidar_intensity_array
    # Actuator feedbacks
    throttle = float(data["Throttle"])
    steering_angle = float(data["Steering"])
    # Wheel encoders
    encoder_ticks = np.fromstring(data["Encoder Ticks"], dtype=int, sep=' ')
    encoder_angles = np.fromstring(data["Encoder Angles"], dtype=float, sep=' ')
    # IPS
    position = np.fromstring(data["Position"], dtype=float, sep=' ')
    # IMU
    orientation_quaternion = np.fromstring(data["Orientation Quaternion"], dtype=float, sep=' ')
    orientation_euler_angles = np.fromstring(data["Orientation Euler Angles"], dtype=float, sep=' ')
    angular_velocity = np.fromstring(data["Angular Velocity"], dtype=float, sep=' ')
    linear_acceleration = np.fromstring(data["Linear Acceleration"], dtype=float, sep=' ')
    # LIDAR
    lidar_scan_rate = float(data["LIDAR Scan Rate"])
    lidar_range_array = np.fromstring(data["LIDAR Range Array"], dtype=float, sep=' ')
    lidar_intensity_array = np.fromstring(data["LIDAR Intensity Array"], dtype=float, sep=' ')
    # Cameras
    front_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data["Front Camera Image"])))), cv2.COLOR_RGB2BGR)
    rear_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data["Rear Camera Image"])))), cv2.COLOR_RGB2BGR)
    if verbose:
        # Monitor vehicle sensor data
        print('Throttle: {}'.format(throttle))
        print('Steering: {}'.format(steering_angle))
        print('Encoder Ticks:  {} {}'.format(encoder_ticks[0],encoder_ticks[1]))
        print('Encoder Angles: {} {}'.format(encoder_angles[0],encoder_angles[1]))
        print('Position: {} {} {}'.format(position[0],position[1],position[2]))
        print('Orientation [Quaternion]: {} {} {} {}'.format(orientation_quaternion[0],orientation_quaternion[1],orientation_quaternion[2],orientation_quaternion[3]))
        print('Orientation [Euler Angles]: {} {} {}'.format(orientation_euler_angles[0],orientation_euler_angles[1],orientation_euler_angles[2]))
        print('Angular Velocity: {} {} {}'.format(angular_velocity[0],angular_velocity[1],angular_velocity[2]))
        print('Linear Acceleration: {} {} {}'.format(linear_acceleration[0],linear_acceleration[1],linear_acceleration[2]))
        print('LIDAR Scan Rate: {}'.format(lidar_scan_rate))
        print('LIDAR Range Array: \n{}'.format(lidar_range_array))
        print('LIDAR Intensity Array: \n{}'.format(lidar_intensity_array))
        cv2.imshow("Front Camera Preview", front_image)
        cv2.imshow("Rear Camera Preview", rear_image)
        cv2.waitKey(1)

# Generate vehicle control commands [function]
def generate_vehicle_control_commands(throttle_cmd, steering_cmd, headlights_cmd, indicators_cmd, verbose=False):
    global throttle_command, steering_command, headlights_command, indicators_command
    throttle_command = throttle_cmd
    steering_command = steering_cmd
    headlights_command = headlights_cmd
    indicators_command = indicators_cmd
    if verbose:
        # Monitor vehicle control commands
        print('Throttle Command: {}'.format(throttle_command))
        print('Steering Command: {}'.format(steering_command))
        print('Headlights Command: {}'.format(headlights_command))
        print('Indicators Command: {}'.format(indicators_command))

# Retrieve traffic light status from database [function]
def get_tl_status(TLID):
    try:
        connection = sqlite3.connect('database.db')
        cursor = connection.cursor()
        cursor.execute('SELECT status from TrafficLights where id = ?', [TLID])
        status = cursor.fetchone()[0]
        if status == "Red":
            command = 1
        if status == "Yellow":
            command = 2
        if status == "Green":
            command = 3
        cursor.close()
        return command
    except sqlite3.Error as error:
        print("Could not retrieve from database!", error)
    finally:
        if connection:
            connection.close()

# Update vehicle pose in database [function]
def set_vehicle_pose(VID, posX, posY, yaw):
    try:
        connection = sqlite3.connect('database.db')
        connection.execute('UPDATE Vehicles SET posX = ?' ' WHERE id = ?', (posX, VID))
        connection.execute('UPDATE Vehicles SET posY = ?' ' WHERE id = ?', (posY, VID))
        connection.execute('UPDATE Vehicles SET yaw = ?' ' WHERE id = ?', (yaw, VID))
        connection.commit()
    except sqlite3.Error as error:
        print("Could not update database!", error)
    finally:
        if connection:
            connection.close()

# Generate traffic light control commands [function]
def generate_tl_control_commands(tl1_cmd, tl2_cmd, tl3_cmd, tl4_cmd, verbose=False):
    global tl1_command, tl2_command, tl3_command, tl4_command
    tl1_command = tl1_cmd
    tl2_command = tl2_cmd
    tl3_command = tl3_cmd
    tl4_command = tl4_cmd
    if verbose:
        # Monitor traffic light control commands
        print('Traffic Light 1 Command: {}'.format(tl1_command))
        print('Traffic Light 2 Command: {}'.format(tl2_command))
        print('Traffic Light 3 Command: {}'.format(tl3_command))
        print('Traffic Light 4 Command: {}'.format(tl4_command))

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__) # '__main__'

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print('Connected!')

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    if data:
        ####################################################################################################################################
        # VEHICLE
        ####################################################################################################################################

        # Perception
        receive_vehicle_sensor_data(data, verbose=False)

        # Planning

        # Control
        throttle_cmd = 1 # [-1, 1]
        steering_cmd = 1 # [-1, 1]
        headlights_cmd = 1 # [0 = disabled, 1 = low beam, 2 = high beam]
        indicators_cmd = 3 # [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]
        generate_vehicle_control_commands(throttle_cmd, steering_cmd, headlights_cmd, indicators_cmd, verbose=False)

        ####################################################################################################################################
        # SMART CITY
        ####################################################################################################################################

        # Monitoring
        posX = position[0] # Vehicle position x-coordinate (m)
        posY = position[1] # Vehicle position y-coordinate (m)
        yaw = orientation_euler_angles[2]  # Vehicle yaw (rad)
        set_vehicle_pose(1, posX, posY, yaw)

        # Control
        tl1_cmd = get_tl_status(1) # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl2_cmd = get_tl_status(2) # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl3_cmd = get_tl_status(3) # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl4_cmd = get_tl_status(4) # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        generate_tl_control_commands(tl1_cmd, tl2_cmd, tl3_cmd, tl4_cmd, verbose=False)

        ####################################################################################################################################

        try:
            sio.emit('Bridge',
            data={'Throttle': throttle_command.__str__(),
                  'Steering': steering_command.__str__(),
                  'Headlights': headlights_command.__str__(),
                  'Indicators': indicators_command.__str__(),
                  'Traffic Light 1': tl1_command.__str__(),
                  'Traffic Light 2': tl2_command.__str__(),
                  'Traffic Light 3': tl3_command.__str__(),
                  'Traffic Light 4': tl4_command.__str__()})
        except Exception as exception_instance:
            print(exception_instance)

if __name__ == '__main__':
    app = socketio.Middleware(sio, app) # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app) # Deploy as an eventlet WSGI server
