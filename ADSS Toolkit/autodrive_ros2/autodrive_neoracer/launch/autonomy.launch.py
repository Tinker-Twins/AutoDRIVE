import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('neoracer_ros2_driver')

    enable_slam = DeclareLaunchArgument('slam', default_value='false')
    enable_nav = DeclareLaunchArgument('nav', default_value='false')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('osracer_description'),
            'launch', 'osracer_description.launch.py')),
        launch_arguments={
            'start_jsp': 'false',
            'jsp_gui': 'false',
            'use_rviz': 'false',
            'publish_frequency': '100.0',
            'odom_topic': '/odom',
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('osracer_slam'),
            'launch', 'slam_toolbox.launch.py')),
        condition=IfCondition(LaunchConfiguration('slam')),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory('autodrive_neoracer'),
                'config', 'slam_params.yaml'),
        }.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('osracer_navigation'),
                'launch',
                'bringup_launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('nav')),
        launch_arguments={
            'use_namespace': 'False',
            'use_composition': 'False',
            'slam': 'False',
            'map': os.path.join(
                get_package_share_directory('autodrive_neoracer'),
                'maps',
                'map.yaml',
            ),
            'params_file': os.path.join(
                get_package_share_directory('autodrive_neoracer'),
                'config',
                'nav_params.yaml',
            ),
        }.items(),
    )

    twist_bridge = Node(
        package='neoracer_ros2_driver',
        executable='twist_bridge',
        name='twist_bridge_node',
        parameters=[os.path.join(pkg_share, 'config', 'twist_bridge.yaml')],
        output='screen',
    )

    imu_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        remappings=[
            ('imu/data_raw', '/imu/fused'),
            ('imu/data', 'imu_filter'),
        ],
        parameters=[{
            'do_bias_estimation': True,
            'do_adaptive_gain': True,
            'use_mag': False,
            'gain_acc': 0.01,
            'gain_mag': 0.01,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('autodrive_neoracer'),
                'config', 'odom_params.yaml'),
            {
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_footprint',
                'world_frame': 'odom',
                'publish_tf': True,
            },
        ],
    )

    return LaunchDescription([
        enable_slam,
        enable_nav,
        description,
        twist_bridge,
        imu_filter,
        ekf,
        slam,
        nav,
    ])
