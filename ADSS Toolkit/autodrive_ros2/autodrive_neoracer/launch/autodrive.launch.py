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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    driver_config = os.path.join(
        get_package_share_directory('neoracer_ros2_driver'), 'config')

    return LaunchDescription([
        Node(
            package='neoracer_ros2_driver',
            executable='mux_node',
            name='mux_node',
            output='screen',
            parameters=[os.path.join(driver_config, 'mux.yaml')],
        ),
        Node(
            package='neoracer_ros2_driver',
            executable='throttle_node',
            name='throttle_node',
            output='screen',
            parameters=[os.path.join(driver_config, 'throttle.yaml')],
        ),
        Node(
            package='neoracer_ros2_driver',
            executable='gamepad_node',
            name='gamepad_node',
            output='screen',
            parameters=[os.path.join(driver_config, 'gamepad.yaml')],
        ),
        Node(
            package='autodrive_neoracer',
            executable='autodrive_bridge',
            name='autodrive_bridge',
            output='screen',
        ),
        DeclareLaunchArgument('inference', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('neoracer_ros2_driver'),
                'launch', 'inference.launch.py')),
            condition=IfCondition(LaunchConfiguration('inference')),
        ),
    ])
