import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # AutoDRIVE runs the REAL mux and throttle nodes from neoracer_ros2_driver
    # with the car's own configs, so /drive -> mux -> /mux_out -> throttle ->
    # /motor behaves exactly as on hardware (incl. the 0.625 steering cap and
    # the 50 Hz zero-on-stale watchdog). The bridge replaces only the hardware
    # I/O behind /motor.
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
        DeclareLaunchArgument('inference', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('neoracer_ros2_driver'),
                'launch', 'inference.launch.py')),
            condition=IfCondition(LaunchConfiguration('inference')),
        ),
    ])
