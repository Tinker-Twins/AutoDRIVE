#!/usr/bin/env python3

################################################################################

# Copyright (c) 2026, Neobotics Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
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

# NeoRacer digital twin bridge for AutoDRIVE Simulator.
#
# Replaces ONLY the hardware I/O of neoracer_ros2_driver (ESP32 serial, LakiBeam,
# USB camera); the real mux_node and throttle_node from the driver package run
# unmodified alongside it (see sim_twin.launch.py), so the twin exercises the
# same /drive -> mux -> /mux_out -> throttle -> /motor chain as the car:
#
#   Subscribe /motor           ackermann_msgs/AckermannDriveStamped (normalized [-1, 1])
#   Publish   /scan            sensor_msgs/LaserScan   (frame 'laser', RELIABLE like lakibeam1)
#   Publish   /camera/color    sensor_msgs/Image       (frame 'camera_link', encoding 'jpeg')
#   Publish   /imu/fused       sensor_msgs/Imu         (frame 'imu_link', RELIABLE depth 10)
#   Publish   /odom            nav_msgs/Odometry       (odom -> base_footprint, RELIABLE depth 10)
#   Publish   /battery         sensor_msgs/BatteryState (static pack voltage; sim has no battery model)
#   Publish   /battery/voltage std_msgs/Float32         (racecar_neo scalar contract)
#   Publish   /encoder/speed   std_msgs/Float32         (motor-encoder ground speed, m/s)
#   Publish   /rc/channels     std_msgs/Float32MultiArray (10 channels, transmitter-off neutral)
#
# Faithful-twin notes:
# - /motor speed mirrors controller.py motor_to_command(): +/-6 m/s. The real
#   ESP32 closes the speed loop; the simulator throttle is open-loop, so a
#   feedforward + proportional controller on measured speed reproduces the
#   closed-loop behavior. Steering models the wire-to-wheel geometry: /motor
#   0.625 (the throttle cap) = true 30 deg wheel lock (STEERING_WIRE_FULL_LOCK).
# - /scan is limited to 30 Hz and /camera to 60 fps, the LakiBeam scanfreq and
#   camera framerate on the real car (telemetry frames arrive at sim rate).
# - /odom twist.angular is zero, matching the real driver's documented gap:
#   consumers take yaw rate from /imu, the same contract as on hardware.
# - /camera carries JPEG bytes inside sensor_msgs/Image with encoding 'jpeg'
#   (the driver's convention) so racecar-neo's decoder works unchanged.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python
from rclpy.node import Node # ROS 2 node base class
from rclpy.qos import qos_profile_sensor_data # QoS profile matching the hardware driver
from ackermann_msgs.msg import AckermannDriveStamped # Drive command message class
from sensor_msgs.msg import BatteryState, Imu, Image, Joy, LaserScan # Sensor message classes
from nav_msgs.msg import Odometry # Odometry message class
from std_msgs.msg import Float32, Float32MultiArray # Scalar racecar_neo sensor topics

# Python module imports
import socketio # Socket.IO realtime client and server
from gevent import pywsgi # Pure-Python gevent-friendly WSGI server
from geventwebsocket.handler import WebSocketHandler # WebSocket message handler
import numpy as np # Scientific computing
import base64 # Base64 binary-to-text encoding/decoding scheme
import gzip # Gzip de/compression for LIDAR range arrays
import math # Mathematical functions
import os # Operating system interfaces
import signal # Asynchronous event signals
import threading # Thread-based parallelism
import time # Monotonic clocks for output rate limiting
import traceback # Stack trace formatting

################################################################################

# NeoRacer actuation limits (mirror neoracer_ros2_driver controller.py)
MAX_SPEED_MPS = 6.0 # Vehicle speed limit (m/s) - controller.yaml max_speed_mps
SPEED_KP = 0.5 # Proportional gain of the speed controller
# Firmware maps wire degrees +/-30 onto the FULL servo pulse swing, but the
# steering linkage reaches true 30 deg wheel lock at 0.625 of that swing
# (throttle.yaml caps /motor there so the servo never stalls past lock).
# Wire-to-wheel gain is therefore 1/0.625; the clamp below is the mechanical
# stop. /motor 0.625 = wheels at 30 deg = sim steering 1.0.
STEERING_WIRE_FULL_LOCK = 0.625
LIDAR_WIRE_BINS = 1440 # Full-circle bins at 0.25 deg, the LakiBeam wire format
SCAN_RATE_HZ = 30.0 # LakiBeam scanfreq on the real car (lidar.launch.py)
CAMERA_FPS = 60.0 # USB camera framerate on the real car (camera.yaml)
BATTERY_V_MIN = 10.8 # 3S LiPo bounds, matching controller.py charge mapping
BATTERY_V_MAX = 12.6

#########################################################
# TELEMETRY PARSING FUNCTIONS
#########################################################

def floats(s):
    return np.asarray([float(v) for v in s.split()]) # Parse space-separated floats

def parse_array(s):
    # Two wire formats exist: gzip+base64 newline-joined (simulator source builds)
    # and plain space-separated (release binaries). The gzip magic number decides.
    try:
        raw = gzip.decompress(base64.b64decode(s)).decode('utf-8')
        return np.asarray([float('inf') if v == 'inf' else float(v) for v in raw.split('\n') if v])
    except Exception:
        return floats(s)

#########################################################
# ROS 2 MESSAGE GENERATING FUNCTIONS
#########################################################

def create_laser_scan_msg(stamp, ranges):
    # LakiBeam wire format (lakibeam1_scan.cpp): a FULL-CIRCLE array spanning
    # -180..180 deg at 0.25 deg/bin, no-return = inf. Only the 270 deg window
    # (|angle| <= 135) carries returns; the rear wedge is blind. racecar-neo's
    # lidar_real ignores angle metadata and treats ranges[] as the full circle
    # (flip + half-turn roll), so publishing the sim's 270 deg sweep as a bare
    # 1080-bin array stretches every beam angle by 4/3 - the sweep must drop
    # 1:1 into the center bins of the full-circle array instead.
    ls = LaserScan()
    ls.header.stamp = stamp
    ls.header.frame_id = 'laser' # Frame ID matching the hardware LIDAR launch
    ls.angle_min = -math.pi
    ls.angle_max = math.pi
    ls.angle_increment = 2.0 * math.pi / LIDAR_WIRE_BINS
    ls.scan_time = 1.0 / SCAN_RATE_HZ # Time per complete scan at the real scanfreq
    # The simulator captures all rays in the same physics tick - the scan is
    # instantaneous. A nonzero per-ray time makes scan matchers de-skew motion
    # that never happened and smears rotation into the map.
    ls.time_increment = 0.0
    ls.range_min = 0.06 # Minimum sensor range (m)
    ls.range_max = 25.0 # Maximum sensor range (m) per LakiBeam1 specification
    wire = np.full(LIDAR_WIRE_BINS, math.inf, dtype=np.float32)
    start = (LIDAR_WIRE_BINS - ranges.size) // 2
    wire[start:start + ranges.size] = ranges
    ls.ranges = wire.tolist()
    return ls

def create_imu_msg(stamp, orientation_quaternion, angular_velocity, linear_acceleration):
    imu = Imu()
    imu.header.stamp = stamp
    imu.header.frame_id = 'imu_link' # Frame ID matching the hardware driver
    imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = orientation_quaternion
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = angular_velocity
    # A real accelerometer measures SPECIFIC FORCE: at rest it reads +9.81
    # along body-up (gravity reaction). The simulator streams coordinate
    # acceleration (~0 at rest), which starves the complementary filter's
    # gravity estimate into NaN quaternions (measured 2026-08-03) and poisons
    # the EKF yaw. Add gravity rotated into the body frame.
    qx, qy, qz, qw = (float(v) for v in orientation_quaternion)
    imu.linear_acceleration.x = float(linear_acceleration[0]) + 9.81 * 2.0 * (qx * qz - qw * qy)
    imu.linear_acceleration.y = float(linear_acceleration[1]) + 9.81 * 2.0 * (qy * qz + qw * qx)
    imu.linear_acceleration.z = float(linear_acceleration[2]) + 9.81 * (1.0 - 2.0 * (qx * qx + qy * qy))
    return imu

def create_odom_msg(stamp, position, orientation, linear_velocity):
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = 'odom' # Frame IDs matching the hardware driver
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position.x = float(position[0])
    odom.pose.pose.position.y = float(position[1])
    odom.pose.pose.position.z = float(position[2])
    odom.pose.pose.orientation = orientation
    # V1 Linear Velocity is already body-frame (measured 2026-08-03: forward
    # component = speed, lateral ~0 at all headings) - REP-105-correct as-is.
    odom.twist.twist.linear.x = float(linear_velocity[0])
    odom.twist.twist.linear.y = float(linear_velocity[1])
    # twist.angular intentionally zero: the hardware driver's documented contract
    # is that yaw rate comes from /imu, not /odom. A faithful twin keeps the gap.
    return odom

def create_image_msg(stamp, jpeg_bytes):
    img = Image()
    img.header.stamp = stamp
    img.header.frame_id = 'camera_link' # Frame ID matching camera.yaml on the real car
    img.encoding = 'jpeg' # Hardware driver convention: JPEG bytes inside sensor_msgs/Image
    img.data = jpeg_bytes
    img.height = 1
    img.width = len(jpeg_bytes)
    img.step = len(jpeg_bytes)
    return img

#########################################################
# NEORACER DIGITAL TWIN BRIDGE NODE
#########################################################

class SimTwinBridge(Node):
    def __init__(self):
        super().__init__('sim_twin_bridge')
        # Wire convention is REP-103 (positive = left turn): racecar_core's
        # drive_real negates the user-facing right-positive angle before publishing,
        # and mux/throttle pass the sign through unchanged. The simulator's
        # positive steering also turns left, so identity mapping.
        self.declare_parameter('steering_sign', 1.0)
        # Sim has no battery model; publish the real message shape at a fixed
        # pack voltage so dashboard/consumers see the hardware contract.
        self.declare_parameter('battery_voltage', 12.6)
        self.steering_sign = self.get_parameter('steering_sign').value
        self.battery_voltage = self.get_parameter('battery_voltage').value

        # Publishers and subscriber (QoS matching the hardware driver:
        # lakibeam1 publishes /scan RELIABLE depth 1000; controller.py publishes
        # /imu and /odom RELIABLE depth 10 for the EKF, /camera and /battery
        # sensor-data; /motor is the throttle node's BEST_EFFORT output)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 1000)
        self.pub_imu = self.create_publisher(Imu, '/imu/fused', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_camera = self.create_publisher(Image, '/camera/color', qos_profile_sensor_data)
        self.pub_battery = self.create_publisher(BatteryState, '/battery', qos_profile_sensor_data)
        # Scalar racecar_neo sensor topics (driver v0.4.2, contract-sync with
        # MITRacecarNeo). /battery/current is deliberately ABSENT: the OSRbot
        # base has no current shunt, nothing publishes it on the car either.
        self.pub_voltage = self.create_publisher(Float32, '/battery/voltage', qos_profile_sensor_data)
        self.pub_encoder = self.create_publisher(Float32, '/encoder/speed', qos_profile_sensor_data)
        self.pub_rc = self.create_publisher(Float32MultiArray, '/rc/channels', qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, '/motor', self.on_motor, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_battery)

        # Actuation state shared between the ROS executor and Socket.IO threads
        self.lock = threading.Lock()
        self.target_speed = 0.0
        self.steering_cmd = 0.0
        self.actual_speed = 0.0
        self.last_scan_t = 0.0
        self.last_camera_t = 0.0

        # Virtual gamepad: racecar_core gates user code behind the START button.
        # When a lab subscribes to /joy, press START for it automatically.
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.joy_subs_seen = 0
        self.autostart_state = -1
        self.create_timer(1.0, self.autostart_tick)

    def autostart_tick(self):
        count = self.joy_pub.get_subscription_count()
        if count > self.joy_subs_seen:
            self.autostart_state = 2 # new lab attached; press in 2 ticks (init time)
        self.joy_subs_seen = count
        if self.autostart_state > 0:
            self.autostart_state -= 1
        elif self.autostart_state == 0:
            msg = Joy()
            msg.axes = [0.0] * 8
            msg.buttons = [0] * 11
            msg.buttons[7] = 1 # START
            self.joy_pub.publish(msg)
            self.autostart_state = -2 # release on the next tick
        elif self.autostart_state == -2:
            msg = Joy()
            msg.axes = [0.0] * 8
            msg.buttons = [0] * 11
            self.joy_pub.publish(msg)
            self.autostart_state = -1
            self.get_logger().info('virtual gamepad: START pressed for attached lab')

    def on_motor(self, msg):
        # Mirror controller.py motor_to_command() + the firmware's wire-to-wheel
        # steering geometry (see STEERING_WIRE_FULL_LOCK above).
        with self.lock:
            self.target_speed = max(-1.0, min(1.0, msg.drive.speed)) * MAX_SPEED_MPS
            self.steering_cmd = self.steering_sign * max(
                -1.0, min(1.0, msg.drive.steering_angle / STEERING_WIRE_FULL_LOCK))

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

    def throttle(self):
        # Feedforward + proportional speed control: reproduces the ESP32's
        # closed-loop speed behavior on top of the simulator's open-loop throttle
        with self.lock:
            target = self.target_speed
        feedforward = target / MAX_SPEED_MPS
        proportional = SPEED_KP * (target - self.actual_speed) / MAX_SPEED_MPS
        return max(-1.0, min(1.0, feedforward + proportional))

    def publish_telemetry(self, data):
        orientation_quaternion = floats(data['V1 Orientation Quaternion'])
        position = floats(data['V1 Position'])
        if orientation_quaternion.size != 4 or position.size != 3:
            return # First frames arrive before the simulator sensors tick; skip until complete

        stamp = self.get_clock().now().to_msg()
        angular_velocity = floats(data['V1 Angular Velocity'])
        linear_acceleration = floats(data['V1 Linear Acceleration'])
        linear_velocity = floats(data['V1 Linear Velocity']) \
            if 'V1 Linear Velocity' in data else np.zeros(3)
        self.actual_speed = float(np.hypot(linear_velocity[0], linear_velocity[1]))

        # Telemetry frames arrive at sim frame rate (~130 Hz); the real sensors
        # don't. Gate /scan to the LakiBeam's 30 Hz and /camera to 60 fps.
        now = time.monotonic()
        if now - self.last_scan_t >= 1.0 / SCAN_RATE_HZ:
            self.last_scan_t = now
            self.pub_scan.publish(create_laser_scan_msg(
                stamp, parse_array(data['V1 LIDAR Range Array'])))
        imu = create_imu_msg(stamp, orientation_quaternion, angular_velocity, linear_acceleration)
        self.pub_imu.publish(imu)
        self.pub_odom.publish(create_odom_msg(stamp, position, imu.orientation, linear_velocity))
        # Motor-encoder ground speed; and the FlySky channels in their
        # transmitter-off state (failsafe maps to neutral), like a car with no
        # transmitter bound.
        self.pub_encoder.publish(Float32(data=self.actual_speed))
        self.pub_rc.publish(Float32MultiArray(data=[0.0] * 10))
        if now - self.last_camera_t >= 1.0 / CAMERA_FPS:
            self.last_camera_t = now
            self.pub_camera.publish(create_image_msg(
                stamp, base64.b64decode(data['V1 Front Camera Image'])))

#########################################################
# SOCKET.IO SERVER INFRASTRUCTURE
#########################################################

def main():
    # rclpy's default handlers shut down the ROS context but leave the gevent
    # server in the main thread serving headless. Own the signals instead: a
    # stateless bridge has nothing to flush, so exit hard and release the port.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: os._exit(0))
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
    node = SimTwinBridge()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sio = socketio.Server(async_mode='gevent')
    frame_count = [0]

    @sio.on('connect')
    def on_connect(sid, environ):
        node.get_logger().info('AutoDRIVE Simulator connected')

    @sio.on('Bridge')
    def on_bridge(sid, data):
        # The simulator waits for this reply before sending the next frame, so
        # the reply must go out even when a frame fails to parse -- and the
        # failure must be loud: socketio swallows handler exceptions by default.
        if data:
            frame_count[0] += 1
            if frame_count[0] % 1000 == 1:
                node.get_logger().info('frame %d received' % frame_count[0])
            try:
                node.publish_telemetry(data)
            except Exception:
                node.get_logger().error('telemetry frame failed:\n' + traceback.format_exc())
        # V1 Reset and V1 CoSim must be present: in autonomous mode the simulator's
        # OnBridge parses them before throttle/steering whenever the scene wires a
        # ResetManager/CoSimManager, and a missing field kills its handler mid-parse
        # (which stalls the request-reply telemetry loop entirely).
        sio.emit('Bridge', data={
            'V1 Throttle': str(node.throttle()),
            'V1 Steering': str(node.steering_cmd),
            'V1 Reset': 'False',
            'V1 CoSim': '0',
        })

    node.get_logger().info('NeoRacer sim twin listening on :4567 '
                           '(driver v0.4.2 parity: /motor /scan /camera/color /imu/fused '
                           '/odom /battery /battery/voltage /encoder/speed /rc/channels)')
    app = socketio.WSGIApp(sio)
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()

################################################################################

if __name__ == '__main__':
    main()
