"""
YDLIDAR TG30 のドライバを起動する launch.

出力: <namespace>/scan (LaserScan)、frame は frame_id。
ドライバは ydlidar_ros2_driver。付属の ydlidar_launch.py は ROS 1 時代の
引数 (node_executable など) を使っていて Jazzy では動かないので、使わない。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nr27_sensors'), 'config', 'tg30.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='tg30'),
        DeclareLaunchArgument('frame_id', default_value='tg30_link'),
        DeclareLaunchArgument('scan_topic', default_value='/tg30/scan'),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[config, {'frame_id': LaunchConfiguration('frame_id')}],
            remappings=[('scan', LaunchConfiguration('scan_topic'))],
        ),
    ])
