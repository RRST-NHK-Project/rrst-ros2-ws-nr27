"""
LiDAR をまとめて起動する launch (入口).

使い方:
    ros2 launch nr27_sensors lidar.launch.py lidars:=ld19
    ros2 launch nr27_sensors lidar.launch.py lidars:=ld19,tg30,ust30lx

LiDAR ごとに、次の名前で出す (<name> = ld19 / tg30 / ust30lx):
    トピック  /<name>/scan  (sensor_msgs/LaserScan)
    frame     <name>_link
取り付け位置の TF (base_link -> <name>_link) は config/lidar_mount.yaml から流す。
別の TF の流し方 (URDF など) をするときは mount_tf:=false にする。

使う側 (SLAM、壁検知、コストマップなど) は、機種を知らずに、scan_topic の
パラメータで /<name>/scan を選ぶ。機種を増やすときは、<name>.launch.py と
config/<name>.yaml を足して、下の MODELS に名前を足すだけでよい。

事前にドライバのワークスペースを source しておくこと:
    source ~/ros2_ws/install/setup.bash            # ldlidar, urg_node2
    source ~/ydlidar_ros2_ws/install/setup.bash    # ydlidar_ros2_driver
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

MODELS = ('ld19', 'tg30', 'ust30lx')


def mount_tf(name, parent, pose):
    """取り付け位置の static TF (parent -> <name>_link) を作る."""
    args = ['--frame-id', parent, '--child-frame-id', f'{name}_link']
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        args += [f'--{key}', str(pose.get(key, 0.0))]
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{name}_mount_tf',
        arguments=args,
        condition=IfCondition(LaunchConfiguration('mount_tf')),
    )


def launch_setup(context):
    share = get_package_share_directory('nr27_sensors')
    names = [n.strip() for n in
             LaunchConfiguration('lidars').perform(context).split(',')
             if n.strip()]
    unknown = [n for n in names if n not in MODELS]
    if unknown:
        raise RuntimeError(f'未知の lidars: {unknown} (使えるのは {MODELS})')

    with open(os.path.join(share, 'config', 'lidar_mount.yaml')) as f:
        mount = yaml.safe_load(f)

    actions = []
    for name in names:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', f'{name}.launch.py')),
            launch_arguments={
                'namespace': name,
                'frame_id': f'{name}_link',
                'scan_topic': f'/{name}/scan',
            }.items()))
        actions.append(mount_tf(name, mount.get('parent_frame', 'base_link'),
                                mount.get(name, {})))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidars', default_value='ld19',
            description='起動する LiDAR (カンマ区切り): ld19, tg30, ust30lx'),
        DeclareLaunchArgument(
            'mount_tf', default_value='true',
            description='取り付け位置の static TF を流すか'),
        OpaqueFunction(function=launch_setup),
    ])
