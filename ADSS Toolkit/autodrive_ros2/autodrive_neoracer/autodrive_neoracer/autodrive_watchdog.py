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

# Health Monitor and Recovery Watchdog for AutoDRIVE-NeoRacer
#
# This node monitors the AutoDRIVE-NeoRacer ROS 2 graph and telemetry activity.
# Missing required nodes or a failed inference_node trigger recovery of the
# neoracer-teleop service. Missing topic names and interrupted AutoDRIVE data
# are reported to the NeoRacer System Dashboard without triggering recovery.

################################################################################

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # ROS 2 node class
from rclpy.qos import qos_profile_sensor_data # QoS profile matching neoracer_ros2_driver
from std_msgs.msg import Float32 # Float32 message class
from sensor_msgs.msg import Image, Imu, LaserScan # Sensor message classes
from nav_msgs.msg import Odometry # Odometry message class

# Python module imports
import logging # Logging module for structured log messages
import os # Operating system interfaces for file and process management
from pathlib import Path # Object-oriented filesystem paths
import shutil # High-level file operations and shell utilities
import subprocess # Subprocess management for external command execution
import time # Time-related functions for measuring intervals and delays

#########################################################
# WATCHDOG PARAMETERS
#########################################################

# Monitoring intervals and startup allowances
POLL_SECONDS = 5.0
STARTUP_GRACE_SECONDS = 30.0
INFERENCE_GRACE_SECONDS = 180.0

# Service recovery timing
RESTART_COOLDOWN_SECONDS = 30.0
PORT_RELEASE_TIMEOUT_SECONDS = 10.0

# AutoDRIVE connection health
AUTODRIVE_PORT = 4567
AUTODRIVE_DATA_TIMEOUT_SECONDS = 5.0

# Missing nodes trigger recovery
REQUIRED_NODES = {
    'autodrive_bridge',
    'foxglove_bridge',
    'gamepad_node',
    'mux_node',
    'throttle_node',
}

# Missing topic names are reported as warnings
REQUIRED_TOPICS = {
    '/battery',
    '/battery/voltage',
    '/camera/color',
    '/drive',
    '/encoder/speed',
    '/gamepad_drive',
    '/imu/fused',
    '/joy',
    '/motor',
    '/mux_out',
    '/odom',
    '/rc/channels',
    '/scan',
}

#########################################################
# WATCHDOG LOGGING INFRASTRUCTURE
#########################################################

# Create the file log displayed by the NeoRacer System Dashboard.
def file_logger():
    log_dir = Path.home() / 'logs' / 'latest'
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger('autodrive_watchdog')
    logger.setLevel(logging.INFO)
    logger.propagate = False
    if not logger.handlers:
        handler = logging.FileHandler(log_dir / 'watchdog.log')
        handler.setFormatter(logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
        ))
        logger.addHandler(handler)
    return logger

#########################################################
# AUTODRIVE WATCHDOG NODE
#########################################################

class AutoDRIVE_Watchdog(Node):
    def __init__(self):
        super().__init__('autodrive_watchdog')
        self.started_at = time.monotonic()
        self.file_log = file_logger()
        self.last_state = {}
        self.last_restart = 0.0
        self.last_autodrive_data = None

        # Any message from these AutoDRIVE streams proves that the
        # simulator connection is actively delivering vehicle data.
        self.create_subscription(
            Float32, '/encoder/speed', self.autodrive_data_received,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, '/imu/fused', self.autodrive_data_received,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, '/odom', self.autodrive_data_received,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, '/scan', self.autodrive_data_received,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, '/camera/color', self.autodrive_data_received,
            qos_profile_sensor_data,
        )

    def autodrive_data_received(self, _message):
        """Record the most recent AutoDRIVE data stream."""
        self.last_autodrive_data = time.monotonic()

    def report(self, key, message, level=logging.INFO):
        """Log state transitions once to ROS and the dashboard log."""
        if self.last_state.get(key) == message:
            return
        self.last_state[key] = message
        self.file_log.log(level, message)
        if level >= logging.ERROR:
            self.get_logger().error(message)
        elif level >= logging.WARNING:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def service_command(self, action):
        """Return the command to manage the neoracer-teleop service."""
        supervisor_conf = '/etc/supervisor/conf.d/neoracer.conf'
        if os.path.exists(supervisor_conf):
            return [
                'supervisorctl', '-c', supervisor_conf, action,
                'neoracer-teleop',
            ]
        return ['systemctl', action, 'neoracer-teleop']

    def service_is_running(self):
        """Return whether the neoracer-teleop service is running."""
        result = subprocess.run(
            self.service_command('status'),
            capture_output=True,
            text=True,
            check=False,
        )
        if os.path.exists('/etc/supervisor/conf.d/neoracer.conf'):
            return result.returncode == 0 and 'RUNNING' in result.stdout
        return result.returncode == 0

    @staticmethod
    def autodrive_port_is_open():
        """Return whether a TCP listener owns the AutoDRIVE Bridge port."""
        if not shutil.which('ss'):
            # Fail closed: without a socket inspection tool we cannot prove
            # that the port is available, so recovery must not be attempted.
            return True
        result = subprocess.run(
            ['ss', '-ltnH'], capture_output=True, text=True, check=False)
        if result.returncode != 0:
            return True
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) >= 4 and fields[3].endswith(f':{AUTODRIVE_PORT}'):
                return True
        return False

    def close_autodrive_port(self):
        """Stop stale AutoDRIVE Bridge owners and wait until TCP port is released."""
        deadline = time.monotonic() + PORT_RELEASE_TIMEOUT_SECONDS

        # Ask the current listener to terminate cleanly while time remains.
        while self.autodrive_port_is_open() and time.monotonic() < deadline:
            if shutil.which('fuser'):
                subprocess.run(
                    ['fuser', '-k', '-TERM', f'{AUTODRIVE_PORT}/tcp'],
                    capture_output=True,
                    check=False,
                )
            elif shutil.which('pkill'):
                subprocess.run(
                    ['pkill', '-TERM', '-f', 'autodrive_bridge'],
                    capture_output=True,
                    check=False,
                )
            time.sleep(0.25)

        # Force termination only when a graceful stop did not release the port.
        if self.autodrive_port_is_open() and shutil.which('fuser'):
            subprocess.run(
                ['fuser', '-k', '-KILL', f'{AUTODRIVE_PORT}/tcp'],
                capture_output=True,
                check=False,
            )

        # Never restart neoracer-teleop service until the listener is confirmed gone.
        while self.autodrive_port_is_open() and time.monotonic() < deadline:
            time.sleep(0.25)
        return not self.autodrive_port_is_open()

    def restart_service(self, reason):
        """Safely restart neoracer-teleop service after releasing AutoDRIVE Bridge port."""
        now = time.monotonic()
        if now - self.last_restart < RESTART_COOLDOWN_SECONDS:
            return

        self.report(
            'restart',
            f'AutoDRIVE Recovery: Restarting service ({reason})',
            logging.WARNING,
        )

        # Stop the complete launch tree before handling a stale socket.
        stop = subprocess.run(
            self.service_command('stop'), capture_output=True, text=True,
            check=False,
        )
        if stop.returncode != 0:
            self.report(
                'restart_error',
                'AutoDRIVE Recovery Warning: Could not stop neoracer-teleop: '
                + (stop.stderr or stop.stdout).strip(),
                logging.ERROR,
            )

        # Starting with an occupied port would create a restart loop, so abort
        # recovery if port release cannot be verified.
        if not self.close_autodrive_port():
            self.report(
                'restart_error',
                'AutoDRIVE Recovery Error: TCP port {AUTODRIVE_PORT} '
                'remained occupied; neoracer-teleop was not restarted',
                logging.ERROR,
            )
            self.last_restart = now
            return

        # Start neoracer-teleop service only after TCP port 4567 is available.
        start = subprocess.run(
            self.service_command('start'), capture_output=True, text=True,
            check=False,
        )
        if start.returncode != 0:
            self.report(
                'restart_error',
                'AutoDRIVE Recovery Error: Could not start neoracer-teleop: '
                + (start.stderr or start.stdout).strip(),
                logging.ERROR,
            )
        else:
            self.report(
                'restart',
                'AutoDRIVE Recovery OK: Restarted neoracer-teleop successfully',
                logging.INFO,
            )
        self.last_restart = now

    def check(self):
        """Check required AutoDRIVE-NeoRacer nodes and topics once."""
        age = time.monotonic() - self.started_at

        # Allow ROS discovery and launch processes to settle after startup.
        if age < STARTUP_GRACE_SECONDS:
            return

        # Recover immediately when the service manager reports neoracer-teleop stopped.
        if not self.service_is_running():
            self.restart_service('neoracer-teleop is not running')
            return

        # Required runtime nodes are restart-worthy failures.
        nodes = {
            name for name, _namespace in
            self.get_node_names_and_namespaces()
        }
        missing = sorted(REQUIRED_NODES - nodes)
        if missing:
            reason = 'missing node(s): ' + ', '.join(missing)
            self.report(
                'required_nodes',
                'AutoDRIVE Nodes Warning: Noticed ' + reason,
                logging.WARNING,
            )
            self.restart_service(reason)
        else:
            self.report(
                'required_nodes',
                'AutoDRIVE Nodes OK: All required ROS 2 nodes are present',
                logging.INFO,
            )

        # Topic health is informational: graph presence is checked instead of
        # publisher count, and missing topics never restart neoracer-teleop.
        available_topics = {
            name for name, _types in self.get_topic_names_and_types()
        }
        missing_topics = sorted(REQUIRED_TOPICS - available_topics)
        if missing_topics:
            reason = 'missing topic(s): ' + ', '.join(missing_topics)
            self.report(
                'required_topics',
                'AutoDRIVE Topics Warning: Noticed ' + reason,
                logging.WARNING,
            )
        else:
            self.report(
                'required_topics',
                'AutoDRIVE Topics OK: All required ROS 2 topics are advertised',
                logging.INFO,
            )

        # Activity begins only after AutoDRIVE Simulator connects on port 4567.
        if self.last_autodrive_data is None:
            self.report(
                'autodrive_connection',
                'AutoDRIVE Bridge Warning: Waiting for AutoDRIVE Simulator connection '
                'on port 4567; data stream checks are deferred',
                logging.INFO,
            )
            return

        # Interrupted data stream is reported without restarting neoracer-teleop;
        # the external AutoDRIVE connection may simply have been disconnected.
        autodrive_data_age = time.monotonic() - self.last_autodrive_data
        if autodrive_data_age > AUTODRIVE_DATA_TIMEOUT_SECONDS:
            self.report(
                'autodrive_connection',
                'AutoDRIVE Bridge Warning: Data stopped; check the '
                'AutoDRIVE Simulator connection on port 4567',
                logging.WARNING,
            )
            return
        self.report(
            'autodrive_connection',
            'AutoDRIVE Bridge OK: Streaming data on port 4567',
            logging.INFO
        )

        # Model loading and CUDA/TensorRT warm-up can take a few minutes. Do
        # not treat the absent inference node as a failure during that period.
        if age >= INFERENCE_GRACE_SECONDS and 'inference_node' not in nodes:
            reason = (
                'inference_node is missing after the '
                f'{INFERENCE_GRACE_SECONDS:.0f}s startup grace period'
            )
            self.report(
                'inference_node',
                'Inference Node Warning: ' + reason,
                logging.WARNING,
            )
            self.restart_service(reason)
        elif 'inference_node' in nodes:
            self.report('inference_node', 'Inference Node OK: Inference node is present', logging.INFO)

#########################################################
# AUTODRIVE WATCHDOG INFRASTRUCTURE
#########################################################

def main(args=None):
    """Run the AutoDRIVE Watchdog until ROS shuts down."""
    rclpy.init(args=args)
    node = AutoDRIVE_Watchdog()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            node.check()
            time.sleep(POLL_SECONDS)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

################################################################################

if __name__ == '__main__':
    main()
