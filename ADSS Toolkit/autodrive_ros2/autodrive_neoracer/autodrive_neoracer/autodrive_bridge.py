#!/usr/bin/env python3

################################################################################

# Copyright (c) 2026, Tinker Twins, AutoDRIVE Ecosystem
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

# AutoDRIVE Devkit ROS 2 API for NeoRacer
#
# This node emulates the hardware I/O of neoracer_ros2_driver (ESP32-S3, LakiBeam1,
# USB-Camera); the mux_node and throttle_node from the driver package run unmodified
# alongside it (along with the gamepad_node and inference_node). This way, the digital
# twin exercises the same chain of command as the physical twin of the vehicle:
# /drive --> mux_node --> /mux_out --> throttle_node --> /motor
# Existing dashboards/demos/labs for neoracer & racecar-neo work unchanged.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # ROS 2 node class
from rclpy.qos import qos_profile_sensor_data # QoS profile matching neoracer_ros2_driver
from std_msgs.msg import Float32, Float32MultiArray, String # Standard message classes
from sensor_msgs.msg import BatteryState, Imu, MagneticField, Image, Joy, LaserScan # Sensor message classes
from nav_msgs.msg import Odometry # Odometry message class
from ackermann_msgs.msg import AckermannDriveStamped # Actuation message class

# Python module imports
import socketio # Socket.IO realtime client and server
from gevent import pywsgi # Pure-Python gevent-friendly WSGI server
from geventwebsocket.handler import WebSocketHandler # WebSocket message handler
import numpy as np # Scientific computing
import base64 # Base64 binary-to-text encoding/decoding scheme
import gzip # Inbuilt module to compress and decompress data and files
import math # Mathematical functions
import os # Operating system interfaces
import signal # Asynchronous event signals
import threading # Thread-based parallelism
import traceback # Stack trace formatting
from scipy.spatial.transform import Rotation # Rotation matrix
import queue # FIFO queue

#########################################################
# REAL2SIM PARAMETER MAPPING
#########################################################

# Speed:
# - /motor speed matches neoracer_ros2_driver with a range of +/-6 m/s.
# - The ESP32-S3 on the physical twin closes the speed control loop, while
# this node uses feedforward + feedback (PID) control on measured speed to
# reproduce the physical twin's closed-loop behavior.
#
# Steering:
# - Firmware maps +/-30 degrees steering across the full servo pulse swing.
# - The steering linkage reaches +/-30 deg wheel lock at only 0.625 of the
# full servo pulse swing. This makes the wire-to-wheel gain 1 / 0.625 = 1.6.
# - /motor steer is capped at 0.625 so the servo cannot drive past the
# mechanical steering linkage limit.
# - A 0.625 ms (625 us) servo pulse swing is the maximum pulse-width deviation
# from center used to command full travel in either direction.
# - Steering command = 1.0 <==> /motor = 0.625 <==> wheels at 30 deg lock
#
# Odometry:
# - /odom twist.angular is intentionally zero, matching neoracer_ros2_driver's
# documented behavior.
# - Consumers should obtain yaw rate from /imu, preserving the same interface
# contract as the physical twin.
#
# Camera:
# - /camera contains JPEG bytes in sensor_msgs/Image with encoding 'jpeg',
# matching neoracer_ros2_driver's convention.
# - RGB image with 640x480 pixel resolution.
#
# LIDAR:
# - /scan contains a full circle (360 deg) range measurements spanning -180 to 180 deg
# at 0.25 deg resolution, represented as a 1440-bin array of float32 distances in meters
# matching neoracer_ros2_driver.
# - /scan ranges index 0 points straight behind, and the subsequent elements progress
# counter-clockwise. Only the 270 deg window (|angle| <= 135°) carries returns;
# no-return = inf. The rear sector (between 135° and 225°) is blind; returns inf.
# - racecar-neo's scan is represented as a 1440-bin array of float32 distances in
# centimeters, index 0 points straight ahead, and the subsequent elements progress
# clockwise; no-return = 0.0 & rear blind sector (between 135° and 225°) returns 0.0.

MAX_SPEED_MPS = 6.0 # DBW speed limit for the drive motor (m/s)
SPEED_CTRL_KP = 0.50 # Proportional gain of the speed controller
SPEED_CTRL_KI = 0.01 # Integral gain of the speed controller
SPEED_CTRL_KD = 0.01 # Derivative gain of the speed controller
SPEED_CTRL_KS = 5 # Saturation constant of the speed controller
MAX_STEER_PWM = 0.625 # SBW limit based on servo pulse swing (ms)
LIDAR_SCAN_SIZE = 1440 # 360 deg sweep at 0.25 deg resolution (bins)
LIDAR_SCAN_RATE = 30.0 # LIDAR scan frequency (Hz)
BATTERY_V_MIN = 10.8 # Minimum 3S LiPo voltage (V)
BATTERY_V_MAX = 12.6 # Maximum 3S LiPo voltage (V)

#########################################################
# PID CONTROLLER
#########################################################

class PID_Controller:
    '''
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    '''
    def __init__(self, kP, kI, kD, kS):
        self.kP       = kP # Proportional gain
        self.kI       = kI # Integral gain
        self.kD       = kD # Derivative gain
        self.kS       = kS # Saturation constant (error history buffer size)
        self.err_int  = 0 # Error integral
        self.err_dif  = 0 # Error difference
        self.err_prev = 0 # Previous error
        self.err_hist = queue.Queue(self.kS) # Limited buffer of error history
        self.t_prev   = 0 # Previous time

    def control(self, err, t, kP=None, kI=None, kD=None):
        '''
        Generate PID controller output.
        :param err: Instantaneous error in control variable w.r.t. setpoint
        :param t  : Current timestamp
        :return u : PID controller output
        '''
        kP = self.kP if kP is None else kP
        kI = self.kI if kI is None else kI
        kD = self.kD if kD is None else kD
        # Timestep
        if self.t_prev == 0:
            dt = 1
        else:
            dt = t - self.t_prev
        if dt > 0.0:
            self.err_hist.put(err) # Update error history
            self.err_int += err # Integrate error
            if self.err_hist.full(): # Jacketing logic to prevent integral windup
                self.err_int -= self.err_hist.get() # Rolling FIFO buffer
            self.err_dif = (err - self.err_prev) # Error difference
            u = (kP * err) + (kI * self.err_int * dt) + (kD * self.err_dif / dt) # PID control law
            self.err_prev = err # Update previous error term
            self.t_prev = t # Update timestamp
            return u # Control signal

#########################################################
# ROS 2 MESSAGE GENERATING FUNCTIONS
#########################################################

def create_imu_msg(stamp, orientation, angular_velocity, linear_acceleration):
    imu = Imu()
    imu.header.stamp = stamp
    imu.header.frame_id = 'imu_link' # Frame ID matching the neoracer_ros2_driver
    imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = orientation
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = angular_velocity
    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = linear_acceleration
    return imu

def create_odom_msg(stamp, position, orientation, linear_velocity):
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = 'odom' # Frame ID matching the neoracer_ros2_driver
    odom.child_frame_id = 'base_footprint' # Frame ID matching the neoracer_ros2_driver
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = position
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = orientation
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = linear_velocity
    # odom.twist.twist.angular is intentionally zero matching the neoracer_ros2_driver
    return odom

def create_laserscan_msg(stamp, ranges):
    ls = LaserScan()
    ls.header.stamp = stamp
    ls.header.frame_id = 'laser' # Frame ID matching the neoracer_ros2_driver
    ls.angle_min = -math.pi # Minimum angle of laser scan (-180 deg)
    ls.angle_max = math.pi # Maximum angle of laser scan (180 deg)
    ls.angle_increment = 2.0 * math.pi / LIDAR_SCAN_SIZE # Angular resolution of laser scan (0.25 deg)
    ls.time_increment = (1 / LIDAR_SCAN_RATE) / LIDAR_SCAN_SIZE # Time required to scan 1 degree
    ls.scan_time = 1.0 / LIDAR_SCAN_RATE # Time required to complete a scan
    ls.range_min = 0.0 # Minimum sensor range (m)
    ls.range_max = 25.0 # Maximum sensor range (m)
    scan = np.full(LIDAR_SCAN_SIZE, math.inf, dtype=np.float32)
    start = (LIDAR_SCAN_SIZE - ranges.size) // 2
    scan[start:start+ranges.size] = ranges
    ls.ranges = scan.tolist()
    return ls

def create_image_msg(stamp, jpeg_bytes):
    img = Image()
    img.header.stamp = stamp
    img.header.frame_id = 'camera_link' # Frame ID matching the neoracer_ros2_driver
    img.encoding = 'jpeg' # JPEG encoding matching the neoracer_ros2_driver
    img.data = jpeg_bytes
    img.height = 480
    img.width = 640
    img.step = 0
    img.is_bigendian = 0
    return img

#########################################################
# AUTODRIVE ROS 2 BRIDGE INFRASTRUCTURE
#########################################################

class AutoDRIVE_Bridge(Node):
    def __init__(self):
        super().__init__('autodrive_bridge')

        # Member variables
        self.position = np.zeros(3, dtype=np.float32)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.linear_velocity = np.zeros(3, dtype=np.float32)
        self.angular_velocity = np.zeros(3, dtype=np.float32)
        self.linear_acceleration = np.zeros(3, dtype=np.float32)
        self.target_speed = 0.0
        self.actual_speed = 0.0
        self.throttle_cmd = 0.0
        self.steering_cmd = 0.0
        self.joy_sub_count = 0
        self.joy_btn_state = -1

        # ROS 2 node parameters
        self.declare_parameter('battery_voltage', 12.6)
        self.declare_parameter('magnetic_field_e', -2.8222e-6)  # Magnetic field parameters are based on the WMMHR (2024-2029) model for CU-ICAR
        self.declare_parameter('magnetic_field_n', 22.4546e-6)  # (Lat: 34.81497238768911 deg, Lon: -82.3255659545739 deg, Alt: 299 m) in Tesla
        self.declare_parameter('magnetic_field_u', -42.8711e-6) # expressed in the ENU (East-North-Up) coordinate system.
        self.declare_parameter('speed_ctrl_kP', SPEED_CTRL_KP)
        self.declare_parameter('speed_ctrl_kI', SPEED_CTRL_KI)
        self.declare_parameter('speed_ctrl_kD', SPEED_CTRL_KD)
        self.declare_parameter('throttle_sign', 1.0)
        self.declare_parameter('steering_sign', 1.0)
        self.battery_voltage = self.get_parameter('battery_voltage').value
        self.magnetic_field_e = self.get_parameter('magnetic_field_e').value
        self.magnetic_field_n = self.get_parameter('magnetic_field_n').value
        self.magnetic_field_u = self.get_parameter('magnetic_field_u').value
        self.speed_ctrl_kP = self.get_parameter('speed_ctrl_kP').value
        self.speed_ctrl_kI = self.get_parameter('speed_ctrl_kI').value
        self.speed_ctrl_kD = self.get_parameter('speed_ctrl_kD').value
        self.throttle_sign = self.get_parameter('throttle_sign').value
        self.steering_sign = self.get_parameter('steering_sign').value

        # Speed controller
        self.speed_controller = PID_Controller(self.speed_ctrl_kP,
                                               self.speed_ctrl_kI,
                                               self.speed_ctrl_kD,
                                               SPEED_CTRL_KS) # PID controller object initialized with kP, kI, kD, kS

        # Publishers and subscribers (QoS matching the neoracer_ros2_driver)
        self.create_subscription(AckermannDriveStamped, '/motor', self.callback_motor, qos_profile_sensor_data)
        self.create_subscription(String, '/dotmatrix/text', self.callback_dotmatrix, 10)
        self.pub_rc = self.create_publisher(Float32MultiArray, '/rc/channels', qos_profile_sensor_data)
        self.pub_joy = self.create_publisher(Joy, '/joy', 10)
        self.pub_battery = self.create_publisher(BatteryState, '/battery', qos_profile_sensor_data)
        self.pub_voltage = self.create_publisher(Float32, '/battery/voltage', qos_profile_sensor_data)
        self.pub_encoder = self.create_publisher(Float32, '/encoder/speed', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/imu/fused', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/mag', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_lidar = self.create_publisher(LaserScan, '/scan', 1000)
        self.pub_camera = self.create_publisher(Image, '/camera/color', qos_profile_sensor_data)
        self.create_timer(0.1, self.publish_rc) # 10 Hz
        self.create_timer(0.1, self.publish_joy) # 10 Hz
        self.create_timer(2.0, self.publish_battery) # 0.5 Hz
        self.create_timer(0.05, self.publish_mag) # 20 Hz
        self.create_timer(0.1, self.update_parameters) # 10 Hz

    # ROS 2 subscriber callbacks
    def callback_motor(self, msg):
        self.target_speed = max(-1.0, min(1.0, msg.drive.speed)) * MAX_SPEED_MPS
        feedforward = self.target_speed / MAX_SPEED_MPS
        feedback_pid = self.speed_controller.control(self.target_speed - self.actual_speed,
                                                     self.get_clock().now().nanoseconds * 1e-9,
                                                     self.speed_ctrl_kP,
                                                     self.speed_ctrl_kI,
                                                     self.speed_ctrl_kD) / MAX_SPEED_MPS
        self.throttle_cmd = self.throttle_sign * max(-1.0, min(1.0, feedforward + feedback_pid))
        self.steering_cmd = self.steering_sign * max(-1.0, min(1.0, msg.drive.steering_angle / MAX_STEER_PWM))

    def callback_dotmatrix(self, msg):
        self.get_logger().info('LED Matrix Text: ' + msg.data)

    # ROS 2 publisher functions
    def publish_rc(self):
        self.pub_rc.publish(Float32MultiArray(data=[0.0] * 10))

    def publish_joy(self):
        count = self.pub_joy.get_subscription_count()
        if count > self.joy_sub_count: # New subscriber (e.g., lab attached)
            self.joy_btn_state = 2 # Press in 2 ticks (init time)
        self.joy_sub_count = count
        if self.joy_btn_state > 0:
            self.joy_btn_state -= 1
        elif self.joy_btn_state == 0:
            msg = Joy()
            msg.axes = [0.0] * 8
            msg.buttons = [0] * 11
            msg.buttons[7] = 1 # Press START
            self.pub_joy.publish(msg)
            self.joy_btn_state = -2 # Release on the next tick
        elif self.joy_btn_state == -2:
            msg = Joy()
            msg.axes = [0.0] * 8
            msg.buttons = [0] * 11 # Release START
            self.pub_joy.publish(msg)
            self.joy_btn_state = -1
            self.get_logger().info('Virtual Gamepad: START Triggered!')
        else:
            msg = Joy()
            msg.axes = [0.0] * 8
            msg.buttons = [0] * 11
            self.pub_joy.publish(msg) # Keep publishing to avoid timeout

    def publish_battery(self):
        self.pub_voltage.publish(Float32(data=float(self.battery_voltage)))
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self.battery_voltage
        span = BATTERY_V_MAX - BATTERY_V_MIN
        msg.percentage = min(1.0, max(0.0, (self.battery_voltage - BATTERY_V_MIN) / span))
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_battery.publish(msg)

    def publish_mag(self):
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link' # Frame ID matching the neoracer_ros2_driver
        mag_field_world = np.array([self.magnetic_field_e, self.magnetic_field_n, self.magnetic_field_u], dtype=np.float32)
        world_R_vehicle = Rotation.from_quat(self.orientation).as_matrix()
        mag_field_vehicle = world_R_vehicle.T @ mag_field_world
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = mag_field_vehicle
        self.pub_mag.publish(msg)

    def publish_telemetry(self, data):
        self.position = np.fromstring(data["V1 Position"], dtype=float, sep=' ')
        self.orientation = np.fromstring(data["V1 Orientation Quaternion"], dtype=float, sep=' ')
        # Cache the initial pose to initialize the odometry and TF broadcasting
        if not hasattr(self, 'initial_position') or not hasattr(self, 'initial_orientation'):
            self.initial_position = self.position.copy()
            self.initial_orientation = self.orientation.copy()
        # Compute the relative pose to update the odometry and TF broadcasting
        else:
            initial_orientation_inv = Rotation.from_quat(self.initial_orientation).inv()
            self.position = initial_orientation_inv.apply(self.position - self.initial_position)
            self.orientation = (initial_orientation_inv * Rotation.from_quat(self.orientation)).as_quat()
        # Update and publish telemetry
        self.linear_velocity = np.fromstring(data["V1 Linear Velocity"], dtype=float, sep=' ')
        self.angular_velocity = np.fromstring(data["V1 Angular Velocity"], dtype=float, sep=' ')
        self.linear_acceleration = np.fromstring(data["V1 Linear Acceleration"], dtype=float, sep=' ')
        self.actual_speed = self.linear_velocity[0]
        stamp = self.get_clock().now().to_msg()
        self.pub_encoder.publish(Float32(data=self.actual_speed))
        self.pub_imu.publish(create_imu_msg(stamp, self.orientation, self.angular_velocity, self.linear_acceleration))
        self.pub_odom.publish(create_odom_msg(stamp, self.position, self.orientation, self.linear_velocity))
        self.pub_lidar.publish(create_laserscan_msg(stamp, np.fromstring(gzip.decompress(base64.b64decode(data["V1 LIDAR Range Array"])).decode('utf-8'), sep='\n')))
        self.pub_camera.publish(create_image_msg(stamp, base64.b64decode(data['V1 Front Camera Image'])))

    # ROS 2 parameter updates
    def update_parameters(self):
        names = (
            'battery_voltage',
            'magnetic_field_e',
            'magnetic_field_n',
            'magnetic_field_u',
            'speed_ctrl_kP',
            'speed_ctrl_kI',
            'speed_ctrl_kD',
            'throttle_sign',
            'steering_sign',
        )
        for parameter in self.get_parameters(names):
            setattr(self, parameter.name, parameter.value)

#########################################################
# WEBSOCKET SERVER INFRASTRUCTURE
#########################################################

def main():
    # rclpy's default handlers shut down the ROS 2 context but leave the gevent
    # server in the main thread serving headless. Own the signals instead: a
    # stateless bridge has nothing to flush, so exit hard and release the port.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: os._exit(0))
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))

    # Create the ROS 2 node and spin it in a separate thread so the main thread 
    # can run the gevent server.
    node = AutoDRIVE_Bridge()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    sio = socketio.Server(async_mode='gevent')

    @sio.on('connect')
    def on_connect(sid, environ):
        node.get_logger().info('AutoDRIVE Simulator: Connected!')

    @sio.on('Bridge')
    def on_bridge(sid, data):
        if data:
            try:
                node.publish_telemetry(data)
            except Exception:
                node.get_logger().error('AutoDRIVE Telemetry Failed:\n' + traceback.format_exc())
        sio.emit('Bridge', data={
            'V1 Throttle': str(node.throttle_cmd),
            'V1 Steering': str(node.steering_cmd),
            'V1 Reset': 'False',
            'V1 CoSim': '0',
        })

    node.get_logger().info('AutoDRIVE-NeoRacer ROS 2 Bridge Initialized!')
    app = socketio.WSGIApp(sio)
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()

################################################################################

if __name__ == '__main__':
    main()
