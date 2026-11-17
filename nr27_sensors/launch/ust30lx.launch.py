"""
Hokuyo UST-30LX (Ethernet) のドライバを起動する launch.

出力: <namespace>/scan (LaserScan)、frame は frame_id。
ドライバは urg_node2 (lifecycle node)。起動と同時に configure → activate する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nr27_sensors'), 'config', 'ust30lx.yaml')
    node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='urg_node2',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[config, {'frame_id': LaunchConfiguration('frame_id')}],
        remappings=[('scan', LaunchConfiguration('scan_topic'))],
    )
    configure = RegisterEventHandler(OnProcessStart(
        target_action=node,
        on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE))],
    ))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node,
        start_state='configuring',
        goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))],
    ))
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ust30lx'),
        DeclareLaunchArgument('frame_id', default_value='ust30lx_link'),
        DeclareLaunchArgument('scan_topic', default_value='/ust30lx/scan'),
        node,
        configure,
        activate,
    ])
