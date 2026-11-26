"""urg_node2 と wall_detection_node をまとめて起動する。

    ros2 launch nr27_wall_detection wall_detection.launch.py
    ros2 launch nr27_wall_detection wall_detection.launch.py with_lidar:=false
    ros2 launch nr27_wall_detection wall_detection.launch.py rviz:=false  # rviz なし

rviz (既定で起動) で判定の中身を表示する (rviz/wall_detection.rviz)。
base_frame を base_link にしたときは fixed_frame:=base_link と、
base_link -> laser の TF が要る。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('nr27_wall_detection')
    params = os.path.join(share, 'config', 'wall_detection.yaml')

    with_lidar = LaunchConfiguration('with_lidar')
    rviz_cfg = os.path.join(share, 'rviz', 'wall_detection.rviz')

    urg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('urg_node2'), 'launch',
            'urg_node2.launch.py')),
        condition=IfCondition(with_lidar),
    )

    wall = Node(
        package='nr27_wall_detection',
        executable='wall_detection_node',
        name='wall_detection_node',
        parameters=[params],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='wall_detection_rviz',
        arguments=['-d', rviz_cfg, '-f', LaunchConfiguration('fixed_frame')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('with_lidar', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('fixed_frame', default_value='laser'),
        urg,
        wall,
        rviz,
    ])
