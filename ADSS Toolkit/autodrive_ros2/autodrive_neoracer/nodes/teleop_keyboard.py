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

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # ROS 2 node class
from rclpy.qos import qos_profile_sensor_data # QoS profile matching neoracer_ros2_driver
from ackermann_msgs.msg import AckermannDriveStamped # Actuation message class

# Python module imports
import os # Miscellaneous operating system interfaces
import select # Waiting for I/O completion
import sys # System-specific parameters and functions
if os.name == 'nt':
    import msvcrt # Useful routines from the MS VC++ runtime
else:
    import termios # POSIX style tty control
    import tty # Terminal control functions

################################################################################

# Parameters
SPEED_LIMIT_NORM = 1.0
STEER_LIMIT_NORM = 1.0
SPEED_STEP_SIZE = 0.1
STEER_STEP_SIZE = 0.2

# Information
info = """
----------------------------------------
AutoDRIVE - NeoRacer Teleoperation Panel
----------------------------------------

               Q   W   E
               A   S   D
                   X

W/S : Increase/decrease speed command
D/A : Increase/decrease steer command
Q   : Zero steer
E   : Emergency brake
X   : Force stop and zero steer
Press CTRL+C to quit

NOTE: Press keys within this terminal
----------------------------------------
"""

# Error
error = """
ERROR: Communication Failed!
"""

# Get keyboard key
def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Constrain control commands
def constrain(input, lower_bound, upper_bound):
    if input < lower_bound:
        output = lower_bound
    elif input > upper_bound:
        output = upper_bound
    else:
        output = input
    return output

################################################################################

def main():
    # Settings
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # ROS 2 infrastructure
    rclpy.init()
    node = Node('teleop_keyboard')
    node.declare_parameter('speed_limit_norm', SPEED_LIMIT_NORM)
    node.declare_parameter('steer_limit_norm', STEER_LIMIT_NORM)
    node.declare_parameter('speed_step_size', SPEED_STEP_SIZE)
    node.declare_parameter('steer_step_size', STEER_STEP_SIZE)
    publisher = node.create_publisher(AckermannDriveStamped, '/drive', qos_profile_sensor_data)

    # Initialize
    message = AckermannDriveStamped()
    speed_cmd = 0.0
    steer_cmd = 0.0
    try:
        # Print information
        print(info)

        # Generate control commands
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.speed_limit_norm = node.get_parameter('speed_limit_norm').value
            node.steer_limit_norm = node.get_parameter('steer_limit_norm').value
            node.speed_step_size = node.get_parameter('speed_step_size').value
            node.steer_step_size = node.get_parameter('steer_step_size').value
            key = get_key(settings)
            if key == 'w' :
                speed_cmd = constrain(speed_cmd + node.speed_step_size, -node.speed_limit_norm, node.speed_limit_norm)
            elif key == 's' :
                speed_cmd = constrain(speed_cmd - node.speed_step_size, -node.speed_limit_norm, node.speed_limit_norm)
            elif key == 'a' :
                steer_cmd = constrain(steer_cmd + node.steer_step_size, -node.steer_limit_norm, node.steer_limit_norm)
            elif key == 'd' :
                steer_cmd = constrain(steer_cmd - node.steer_step_size, -node.steer_limit_norm, node.steer_limit_norm)
            elif key == 'q' :
                steer_cmd = 0.0
            elif key == 'e' :
                speed_cmd = 0.0
            elif key == 'x' :
                speed_cmd = 0.0
                steer_cmd = 0.0
            else:
                if (key == '\x03'): # CTRL+C
                    break
            
            # Generate control message
            message.header.stamp = node.get_clock().now().to_msg()
            message.drive.speed = float(speed_cmd)
            message.drive.steering_angle = float(steer_cmd)

            # Publish control message
            publisher.publish(message)

    except Exception as error:
        # Print error
        print(error)

    finally:
        # Generate and publish zero commands
        message.header.stamp = node.get_clock().now().to_msg()
        message.drive.speed = float(0.0)
        message.drive.steering_angle = float(0.0)
        publisher.publish(message)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

################################################################################

if __name__ == '__main__':
    main()