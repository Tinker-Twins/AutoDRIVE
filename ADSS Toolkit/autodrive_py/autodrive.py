#!/usr/bin/env python

# Import libraries
import numpy as np
import base64
from io import BytesIO
from PIL import Image
import cv2

################################################################################

# Nigel class
class Nigel:
    def __init__(self):
        # Nigel data
        self.id                       = None
        self.throttle                 = None
        self.steering                 = None
        self.encoder_ticks            = None
        self.encoder_angles           = None
        self.position                 = None
        self.orientation_quaternion   = None
        self.orientation_euler_angles = None
        self.angular_velocity         = None
        self.linear_acceleration      = None
        self.lidar_scan_rate          = None
        self.lidar_range_array        = None
        self.lidar_intensity_array    = None
        self.front_camera_image       = None
        self.rear_camera_image        = None
        # Nigel commands
        self.throttle_command   = None
        self.steering_command   = None
        self.headlights_command = None
        self.indicators_command = None

    # Parse Nigel sensor data
    def parse_data(self, data, verbose=False):
        # Actuator feedbacks
        self.throttle = float(data[self.id + " Throttle"])
        self.steering = float(data[self.id + " Steering"])
        # Wheel encoders
        self.encoder_ticks = np.fromstring(data[self.id + " Encoder Ticks"], dtype=int, sep=' ')
        self.encoder_angles = np.fromstring(data[self.id + " Encoder Angles"], dtype=float, sep=' ')
        # IPS
        self.position = np.fromstring(data[self.id + " Position"], dtype=float, sep=' ')
        # IMU
        self.orientation_quaternion = np.fromstring(data[self.id + " Orientation Quaternion"], dtype=float, sep=' ')
        self.orientation_euler_angles = np.fromstring(data[self.id + " Orientation Euler Angles"], dtype=float, sep=' ')
        self.angular_velocity = np.fromstring(data[self.id + " Angular Velocity"], dtype=float, sep=' ')
        self.linear_acceleration = np.fromstring(data[self.id + " Linear Acceleration"], dtype=float, sep=' ')
        # LIDAR
        self.lidar_scan_rate = float(data[self.id + " LIDAR Scan Rate"])
        self.lidar_range_array = np.fromstring(data[self.id + " LIDAR Range Array"], dtype=float, sep=' ')
        self.lidar_intensity_array = np.fromstring(data[self.id + " LIDAR Intensity Array"], dtype=float, sep=' ')
        # Cameras
        self.front_camera_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data[self.id + " Front Camera Image"])))), cv2.COLOR_RGB2BGR)
        self.rear_camera_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data[self.id + " Rear Camera Image"])))), cv2.COLOR_RGB2BGR)
        if verbose:
            print('\n--------------------------------')
            print('Receive Data from Nigel: ' + self.id)
            print('--------------------------------\n')
            # Monitor Nigel data
            print('Throttle: {}'.format(self.throttle))
            print('Steering: {}'.format(self.steering))
            print('Encoder Ticks:  {} {}'.format(self.encoder_ticks[0],self.encoder_ticks[1]))
            print('Encoder Angles: {} {}'.format(self.encoder_angles[0],self.encoder_angles[1]))
            print('Position: {} {} {}'.format(self.position[0],self.position[1],self.position[2]))
            print('Orientation [Quaternion]: {} {} {} {}'.format(self.orientation_quaternion[0],self.orientation_quaternion[1],self.orientation_quaternion[2],self.orientation_quaternion[3]))
            print('Orientation [Euler Angles]: {} {} {}'.format(self.orientation_euler_angles[0],self.orientation_euler_angles[1],self.orientation_euler_angles[2]))
            print('Angular Velocity: {} {} {}'.format(self.angular_velocity[0],self.angular_velocity[1],self.angular_velocity[2]))
            print('Linear Acceleration: {} {} {}'.format(self.linear_acceleration[0],self.linear_acceleration[1],self.linear_acceleration[2]))
            print('LIDAR Scan Rate: {}'.format(self.lidar_scan_rate))
            print('LIDAR Range Array: \n{}'.format(self.lidar_range_array))
            print('LIDAR Intensity Array: \n{}'.format(self.lidar_intensity_array))
            cv2.imshow(self.id + ' Front Camera Preview', cv2.resize(self.front_camera_image, (640, 360)))
            cv2.imshow(self.id + ' Rear Camera Preview', cv2.resize(self.rear_camera_image, (640, 360)))
            cv2.waitKey(1)

    # Generate Nigel control commands
    def generate_commands(self, verbose=False):
        if verbose:
            print('\n-------------------------------')
            print('Transmit Data to Nigel: ' + self.id)
            print('-------------------------------\n')
            # Monitor Nigel control commands
            print('Throttle Command: {}'.format(self.throttle_command))
            print('Steering Command: {}'.format(self.steering_command))
            if self.headlights_command == 0:
                headlights_cmd_str = 'Disabled'
            elif self.headlights_command == 1:
                headlights_cmd_str = 'Low Beam'
            elif self.headlights_command == 2:
                headlights_cmd_str = 'High Beam'
            else:
                headlights_cmd_str = 'Invalid'
            print('Headlights Command: {}'.format(headlights_cmd_str))
            if self.indicators_command == 0:
                indicators_cmd_str = 'Disabled'
            elif self.indicators_command == 1:
                indicators_cmd_str = 'Left Turn Indicator'
            elif self.indicators_command == 2:
                indicators_cmd_str = 'Right Turn Indicator'
            elif self.indicators_command == 3:
                indicators_cmd_str = 'Hazard Indicator'
            else:
                indicators_cmd_str = 'Invalid'
            print('Indicators Command: {}'.format(indicators_cmd_str))
        return {str(self.id) + ' Throttle': str(self.throttle_command), str(self.id) + ' Steering': str(self.steering_command), str(self.id) + ' Headlights': str(self.headlights_command), str(self.id) + ' Indicators': str(self.indicators_command)}

################################################################################

# F1TENTH class
class F1TENTH:
    def __init__(self):
        # F1TENTH data
        self.id                       = None
        self.throttle                 = None
        self.steering                 = None
        self.encoder_ticks            = None
        self.encoder_angles           = None
        self.position                 = None
        self.orientation_quaternion   = None
        self.orientation_euler_angles = None
        self.angular_velocity         = None
        self.linear_acceleration      = None
        self.lidar_scan_rate          = None
        self.lidar_range_array        = None
        self.lidar_intensity_array    = None
        self.front_camera_image       = None
        # F1TENTH commands
        self.throttle_command   = None
        self.steering_command   = None
    
    # Parse F1TENTH sensor data
    def parse_data(self, data, verbose=False):
        # Actuator feedbacks
        self.throttle = float(data[self.id + " Throttle"])
        self.steering = float(data[self.id + " Steering"])
        # Wheel encoders
        self.encoder_ticks = np.fromstring(data[self.id + " Encoder Ticks"], dtype=int, sep=' ')
        self.encoder_angles = np.fromstring(data[self.id + " Encoder Angles"], dtype=float, sep=' ')
        # IPS
        self.position = np.fromstring(data[self.id + " Position"], dtype=float, sep=' ')
        # IMU
        self.orientation_quaternion = np.fromstring(data[self.id + " Orientation Quaternion"], dtype=float, sep=' ')
        self.orientation_euler_angles = np.fromstring(data[self.id + " Orientation Euler Angles"], dtype=float, sep=' ')
        self.angular_velocity = np.fromstring(data[self.id + " Angular Velocity"], dtype=float, sep=' ')
        self.linear_acceleration = np.fromstring(data[self.id + " Linear Acceleration"], dtype=float, sep=' ')
        # LIDAR
        self.lidar_scan_rate = float(data[self.id + " LIDAR Scan Rate"])
        self.lidar_range_array = np.fromstring(data[self.id + " LIDAR Range Array"], dtype=float, sep=' ')
        self.lidar_intensity_array = np.fromstring(data[self.id + " LIDAR Intensity Array"], dtype=float, sep=' ')
        # Cameras
        self.front_camera_image = cv2.cvtColor(np.asarray(Image.open(BytesIO(base64.b64decode(data[self.id + " Front Camera Image"])))), cv2.COLOR_RGB2BGR)
        if verbose:
            print('\n--------------------------------')
            print('Receive Data from F1TENTH: ' + self.id)
            print('--------------------------------\n')
            # Monitor F1TENTH data
            print('Throttle: {}'.format(self.throttle))
            print('Steering: {}'.format(self.steering))
            print('Encoder Ticks:  {} {}'.format(self.encoder_ticks[0],self.encoder_ticks[1]))
            print('Encoder Angles: {} {}'.format(self.encoder_angles[0],self.encoder_angles[1]))
            print('Position: {} {} {}'.format(self.position[0],self.position[1],self.position[2]))
            print('Orientation [Quaternion]: {} {} {} {}'.format(self.orientation_quaternion[0],self.orientation_quaternion[1],self.orientation_quaternion[2],self.orientation_quaternion[3]))
            print('Orientation [Euler Angles]: {} {} {}'.format(self.orientation_euler_angles[0],self.orientation_euler_angles[1],self.orientation_euler_angles[2]))
            print('Angular Velocity: {} {} {}'.format(self.angular_velocity[0],self.angular_velocity[1],self.angular_velocity[2]))
            print('Linear Acceleration: {} {} {}'.format(self.linear_acceleration[0],self.linear_acceleration[1],self.linear_acceleration[2]))
            print('LIDAR Scan Rate: {}'.format(self.lidar_scan_rate))
            print('LIDAR Range Array: \n{}'.format(self.lidar_range_array))
            print('LIDAR Intensity Array: \n{}'.format(self.lidar_intensity_array))
            cv2.imshow(self.id + ' Front Camera Preview', cv2.resize(self.front_camera_image, (640, 360)))
            cv2.waitKey(1)

    # Generate F1TENTH control commands
    def generate_commands(self, verbose=False):
        if verbose:
            print('\n-------------------------------')
            print('Transmit Data to F1TENTH: ' + self.id)
            print('-------------------------------\n')
            # Monitor F1TENTH control commands
            print('Throttle Command: {}'.format(self.throttle_command))
            print('Steering Command: {}'.format(self.steering_command))
        return {str(self.id) + ' Throttle': str(self.throttle_command), str(self.id) + ' Steering': str(self.steering_command)}

################################################################################

# Traffic light class
class TrafficLight:
    def __init__(self):
        # Traffic light data
        self.id    = None
        self.state = None
        # Traffic light command
        self.command = None

    # Parse traffic light data
    def parse_data(self, data, verbose=False):
        # Traffic light state
        self.state = int(data[self.id + " State"])
        if verbose:
            print('\n--------------------------------------')
            print('Receive Data from Traffic Light: ' + self.id)
            print('--------------------------------------\n')
            # Monitor traffic light data
            if self.state == 0:
                state_str = 'Disabled'
            elif self.state == 1:
                state_str = 'Red'
            elif self.state == 2:
                state_str = 'Yellow'
            elif self.state == 3:
                state_str = 'Green'
            else:
                state_str = 'Invalid'
            print('Traffic Light State: {}'.format(state_str))

    # Generate traffic light control commands
    def generate_commands(self, verbose=False):
        if verbose:
            print('\n-------------------------------------')
            print('Transmit Data to Traffic Light: ' + self.id)
            print('-------------------------------------\n')
            # Monitor traffic light control commands
            if self.command == 0:
                command_str = 'Disabled'
            elif self.command == 1:
                command_str = 'Red'
            elif self.command == 2:
                command_str = 'Yellow'
            elif self.command == 3:
                command_str = 'Green'
            else:
                command_str = 'Invalid'
            print('Traffic Light Command: {}'.format(command_str))
        return {str(self.id) + ' State': str(self.command)}
