"""
LD19 (LDROBOT) のドライバを起動する launch.

出力: <namespace>/scan (LaserScan)、frame は frame_id。
ドライバは ldrobot-lidar-ros2 (ldlidar_component)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

NODE_NAME = 'ldlidar_node'


def launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    config = os.path.join(
        get_package_share_directory('nr27_sensors'), 'config', 'ld19.yaml')

    # ドライバは lifecycle node。コンテナに載せて、lifecycle_manager で有効化する
    # (nr26 の ldlidar_bringup.launch.py と同じ方式)
    container = ComposableNodeContainer(
        name='ld19_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='ldlidar_component',
                plugin='ldlidar::LdLidarComponent',
                name=NODE_NAME,
                namespace=namespace,
                parameters=[config, {'lidar.frame_id': frame_id,
                                     'autostart': True}],
                remappings=[('~/scan', scan_topic)],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_ld19',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': True,
                     'node_names': [NODE_NAME]}],
    )
    return [container, lifecycle_manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ld19'),
        DeclareLaunchArgument('frame_id', default_value='ld19_link'),
        DeclareLaunchArgument('scan_topic', default_value='/ld19/scan'),
        OpaqueFunction(function=launch_setup),
    ])
