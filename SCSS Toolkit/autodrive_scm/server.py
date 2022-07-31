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

################################################################################

# Parse vehicle sensor data
def parse_vehicle_data(data, verbose=False):
    global steering, throttle, encoder_ticks, encoder_angles, position, orientation_quaternion, orientation_euler_angles, angular_velocity, linear_acceleration, lidar_range_array, lidar_intensity_array, front_camera_image, rear_camera_image
    # Actuator feedbacks
    throttle = float(data["V1 Throttle"])
    steering = float(data["V1 Steering"])
    # Wheel encoders
    encoder_ticks = np.fromstring(data["V1 Encoder Ticks"], dtype=int, sep=' ')
    encoder_angles = np.fromstring(data["V1 Encoder Angles"], dtype=float, sep=' ')
    # IPS
    position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
    # IMU
    orientation_quaternion = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
    orientation_euler_angles = np.fromstring(data["V1 Orientation Euler Angles"], dtype=float, sep=' ')
    angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
    linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
    # LIDAR
    lidar_scan_rate = float(data["V1 LIDAR Scan Rate"])
    lidar_range_array = np.fromstring(data["V1 LIDAR Range Array"], dtype=float, sep=' ')
    lidar_intensity_array = np.fromstring(data["V1 LIDAR Intensity Array"], dtype=float, sep=' ')
    # Cameras
    front_camera_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Front Camera Image"])))), cv2.COLOR_RGB2BGR)
    rear_camera_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data["V1 Rear Camera Image"])))), cv2.COLOR_RGB2BGR)
    if verbose:
        print('\n--------------------------------')
        print('Receive Data from Vehicle: V1')
        print('--------------------------------\n')
        # Monitor vehicle data
        print('Throttle: {}'.format(throttle))
        print('Steering: {}'.format(steering))
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
        cv2.imshow("Front Camera Preview", front_camera_image)
        cv2.imshow("Rear Camera Preview", rear_camera_image)
        cv2.waitKey(1)

################################################################################

# Generate vehicle control commands
def generate_vehicle_commands(throttle_cmd, steering_cmd, headlights_cmd, indicators_cmd, verbose=False):
    global throttle_command, steering_command, headlights_command, indicators_command
    throttle_command = throttle_cmd
    steering_command = steering_cmd
    headlights_command = headlights_cmd
    indicators_command = indicators_cmd
    if verbose:
        print('\n-------------------------------')
        print('Transmit Data to Vehicle: V1')
        print('-------------------------------\n')
        # Monitor vehicle control commands
        print('Throttle Command: {}'.format(throttle_command))
        print('Steering Command: {}'.format(steering_command))
        print('Headlights Command: {}'.format(headlights_command))
        print('Indicators Command: {}'.format(indicators_command))

################################################################################

# Parse traffic light data
def parse_tl_data(data, verbose=False):
    # Traffic light state
    tl1_state = int(data["TL1 State"])
    tl2_state = int(data["TL2 State"])
    tl3_state = int(data["TL3 State"])
    tl4_state = int(data["TL4 State"])
    tl_states = [tl1_state, tl2_state, tl3_state, tl4_state]
    if verbose:
        for i in range(len(tl_states)):
            print('\n--------------------------------------')
            print('Receive Data from Traffic Light: TL' + str(i+1))
            print('--------------------------------------\n')
            # Monitor traffic light data
            print('Traffic Light State: {}'.format(tl_states[i]))

################################################################################

# Generate traffic light control commands
def generate_tl_commands(tl1_cmd, tl2_cmd, tl3_cmd, tl4_cmd, verbose=False):
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

################################################################################

# Update vehicle data (pose) in database
def set_vehicle_data_db(VID, pos_x, pos_y, yaw):
    try:
        connection = sqlite3.connect('database.db')
        connection.execute('UPDATE Vehicles SET pos_x = ?' ' WHERE id = ?', (pos_x, VID))
        connection.execute('UPDATE Vehicles SET pos_y = ?' ' WHERE id = ?', (pos_y, VID))
        connection.execute('UPDATE Vehicles SET yaw = ?' ' WHERE id = ?', (yaw, VID))
        connection.commit()
    except sqlite3.Error as error:
        print("Could not update database!", error)
    finally:
        if connection:
            connection.close()

################################################################################

# Retrieve traffic light data (state) from database
def get_tl_data_db(TLID):
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

################################################################################

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
        parse_vehicle_data(data, verbose=False)

        # Planning

        # Control
        throttle_cmd = 1 # [-1, 1]
        steering_cmd = 1 # [-1, 1]
        headlights_cmd = 1 # [0 = disabled, 1 = low beam, 2 = high beam]
        indicators_cmd = 3 # [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]
        generate_vehicle_commands(throttle_cmd, steering_cmd, headlights_cmd, indicators_cmd, verbose=False)

        ####################################################################################################################################
        # SMART CITY
        ####################################################################################################################################

        # Monitoring
        pos_x = position[0] # Vehicle position x-coordinate (m)
        pos_y = position[1] # Vehicle position y-coordinate (m)
        yaw   = orientation_euler_angles[2]  # Vehicle yaw (rad)
        set_vehicle_data_db(1, pos_x, pos_y, yaw)
        parse_tl_data(data, verbose=False)

        # Control
        tl1_cmd = get_tl_data_db(1) # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl2_cmd = get_tl_data_db(2) # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl3_cmd = get_tl_data_db(3) # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        tl4_cmd = get_tl_data_db(4) # [0 = disabled, 1 = red, 2 = yellow, 3 = green]
        generate_tl_commands(tl1_cmd, tl2_cmd, tl3_cmd, tl4_cmd, verbose=False)

        ####################################################################################################################################

        try:
            sio.emit('Bridge',
            data={'V1 Throttle': throttle_command.__str__(),
                  'V1 Steering': steering_command.__str__(),
                  'V1 Headlights': headlights_command.__str__(),
                  'V1 Indicators': indicators_command.__str__(),
                  'TL1 State': tl1_command.__str__(),
                  'TL2 State': tl2_command.__str__(),
                  'TL3 State': tl3_command.__str__(),
                  'TL4 State': tl4_command.__str__()})
        except Exception as exception_instance:
            print(exception_instance)

################################################################################

if __name__ == '__main__':
    app = socketio.Middleware(sio, app) # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app) # Deploy as an eventlet WSGI server
