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

# Gap-following local planner demo against the NeoRacer driver interface.
#
# This node knows nothing about AutoDRIVE: it subscribes /scan and /odom and
# publishes /drive, exactly like autonomy code on the physical NeoRacer.
#
# Control pipeline, executed per scan:
#   1. PREPROCESS        clip ranges, patch invalid returns
#   2. DISPARITY EXTEND  inflate obstacle edges by the half-width of the car,
#                        so the planner never picks a gap the body cannot fit
#                        through (this is what makes it a *local planner*
#                        rather than a reactive reflex)
#   3. PICK GAP          deepest wide-enough gap in the planning window,
#                        scored by depth and centeredness
#   4. LOOKAHEAD TARGET  a goal point down the chosen ray; pure-pursuit
#                        geometry converts it to a feedforward steering angle
#   5. PD STEERING       proportional-derivative tracking of the target
#                        heading, damping the oscillation a pure reflex shows
#   6. SPEED PLAN        curvature-aware speed with forward-clearance ceiling
#                        and slew-rate limiting (no instant jumps)

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python
from rclpy.node import Node # ROS 2 node base class
from rclpy.qos import qos_profile_sensor_data # QoS profile matching the hardware driver
from ackermann_msgs.msg import AckermannDriveStamped # Drive command message class
from sensor_msgs.msg import LaserScan # Laser scan message class
from nav_msgs.msg import Odometry # Odometry message class

# Python module imports
import numpy as np # Scientific computing
import math # Mathematical functions

################################################################################

# Vehicle geometry (NeoRacer)
WHEELBASE = 0.288 # Vehicle wheelbase (m)
HALF_WIDTH = 0.17 # Half of vehicle width + safety margin (m)
MAX_STEER_RAD = math.radians(30.0) # Steering limit, matching controller.py

# Planner tuning
PLAN_WINDOW_RAD = math.radians(100.0) # Plan within +/-100 deg of straight ahead
CLIP_RANGE = 10.0 # Ranges beyond this do not influence planning (m)
DISPARITY_THRESH = 0.40 # Range jump treated as an obstacle edge (m)
MIN_GAP_DEPTH = 1.2 # Gaps shallower than this are not drivable (m)
CENTER_BIAS = 0.15 # Score bonus for gaps requiring less turning

# Controller tuning (race profile)
STEER_KP = 1.10 # Proportional gain on heading error
STEER_KD = 0.16 # Derivative gain on heading error (more damping at race speed)
SPEED_MAX = 1.00 # Full send on straights, normalized [-1, 1]
SPEED_MIN = 0.22 # Floor command while manoeuvring
CLEAR_FOR_MAX = 6.0 # Forward clearance granting full speed (m)
CURV_SLOWDOWN = 1.2 # Speed penalty per unit of commanded curvature
ACCEL_SLEW = 2.0 # Max speed-command increase per second
BRAKE_SLEW = 4.5 # Max speed-command decrease per second (brake harder than launch)
STOP_TIME = 0.35 # Seconds of travel reserved for the emergency stop
STOP_FLOOR = 0.50 # Minimum emergency-stop clearance (m)

################################################################################

class GapFollower(Node):
    def __init__(self):
        super().__init__('wall_avoid_demo')
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)

        self.measured_speed = 0.0
        self.prev_heading_error = 0.0
        self.prev_speed_cmd = 0.0
        self.prev_stamp = None

    def on_odom(self, msg):
        self.measured_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    #########################################################
    # PLANNING STAGES
    #########################################################

    def preprocess(self, scan):
        ranges = np.asarray(scan.ranges, dtype=float)
        ranges[~np.isfinite(ranges)] = scan.range_max # Treat no-return as max range
        ranges = np.clip(ranges, 0.0, CLIP_RANGE)
        angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
        window = np.abs(angles) <= PLAN_WINDOW_RAD # Plan only within the forward window
        return ranges[window], angles[window], scan.angle_increment

    def disparity_extend(self, ranges, increment):
        # Inflate the closer side of each range discontinuity by the angular
        # width the car body occupies at that distance, so gap selection
        # accounts for vehicle width instead of treating the car as a point.
        extended = ranges.copy()
        jumps = np.abs(np.diff(ranges))
        for i in np.nonzero(jumps > DISPARITY_THRESH)[0]:
            near = min(ranges[i], ranges[i + 1])
            n_mask = int(math.atan2(HALF_WIDTH, max(near, 0.1)) / increment) + 1
            if ranges[i] < ranges[i + 1]: # Obstacle edge on the left of the jump
                extended[i + 1:i + 1 + n_mask] = np.minimum(extended[i + 1:i + 1 + n_mask], near)
            else: # Obstacle edge on the right of the jump
                extended[max(0, i - n_mask + 1):i + 1] = np.minimum(extended[max(0, i - n_mask + 1):i + 1], near)
        return extended

    def pick_target(self, ranges, angles):
        # Deepest drivable ray, scored by depth with a mild centeredness bonus
        # so the planner prefers the gap requiring the least turning when
        # depths are comparable.
        drivable = ranges > MIN_GAP_DEPTH
        if not np.any(drivable):
            i = int(np.argmax(ranges)) # Nothing drivable: aim at the least-bad ray
            return angles[i], ranges[i]
        score = ranges * (1.0 + CENTER_BIAS * np.cos(angles)) * drivable
        i = int(np.argmax(score))
        return angles[i], ranges[i]

    #########################################################
    # CONTROL STAGES
    #########################################################

    def steer(self, target_angle, target_depth, dt):
        # Feedforward: pure-pursuit curvature toward a lookahead point on the
        # chosen ray. Lookahead grows with speed for stability, shrinks when
        # the gap is shallow.
        lookahead = float(np.clip(0.8 + 0.5 * self.measured_speed, 0.8, min(3.5, target_depth)))
        curvature = 2.0 * math.sin(target_angle) / lookahead
        feedforward = math.atan(WHEELBASE * curvature)

        # PD correction on heading error damps the weave a pure reflex shows
        derivative = (target_angle - self.prev_heading_error) / dt if dt > 0 else 0.0
        self.prev_heading_error = target_angle
        steer_rad = STEER_KP * feedforward + STEER_KD * derivative

        return float(np.clip(steer_rad / MAX_STEER_RAD, -1.0, 1.0)), curvature

    def speed(self, ahead, curvature, dt):
        # Emergency-stop distance scales with how fast the car is actually going
        if ahead < max(STOP_FLOOR, STOP_TIME * self.measured_speed):
            self.prev_speed_cmd = 0.0
            return 0.0
        clearance_term = SPEED_MAX * min(1.0, ahead / CLEAR_FOR_MAX)
        curvature_term = SPEED_MAX / (1.0 + CURV_SLOWDOWN * abs(curvature))
        cmd = max(SPEED_MIN, min(clearance_term, curvature_term))
        # Asymmetric slew: launch progressively, brake decisively
        cmd = float(np.clip(cmd, self.prev_speed_cmd - BRAKE_SLEW * dt,
                            self.prev_speed_cmd + ACCEL_SLEW * dt))
        self.prev_speed_cmd = cmd
        return cmd

    #########################################################
    # PER-SCAN CONTROL LOOP
    #########################################################

    def on_scan(self, scan):
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        dt = (stamp - self.prev_stamp) if self.prev_stamp is not None else 0.0
        self.prev_stamp = stamp

        ranges, angles, increment = self.preprocess(scan)
        extended = self.disparity_extend(ranges, increment)
        target_angle, target_depth = self.pick_target(extended, angles)

        # Physical clearance comes from the RAW scan: the disparity-extended copy
        # encodes "drivable for the car's width", which understates true distance
        # to collision and would cap speed at a crawl on open straights.
        ahead = float(np.min(ranges[np.abs(angles) < math.radians(12)]))
        steer_cmd, curvature = self.steer(target_angle, target_depth, dt)
        speed_cmd = self.speed(ahead, curvature, dt)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed_cmd
        msg.drive.steering_angle = steer_cmd # REP-103 throughout: /drive wire positive = left
        self.pub.publish(msg)
        self.get_logger().info(
            f'gap {math.degrees(target_angle):+5.1f}deg depth {target_depth:4.1f}m | '
            f'ahead {ahead:4.2f}m v {self.measured_speed:4.2f}m/s | '
            f'cmd speed {speed_cmd:+.2f} steer {steer_cmd:+.2f}',
            throttle_duration_sec=0.5)

################################################################################

def main():
    rclpy.init()
    rclpy.spin(GapFollower())

################################################################################

if __name__ == '__main__':
    main()
