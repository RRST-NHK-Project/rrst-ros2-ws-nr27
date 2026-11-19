"""
LiDAR と既知のフィールド地図で自己位置を出す launch (map_server + AMCL).

実機 (UST-30LX):
    ros2 launch nr27_localization localization.launch.py x:=-4.1 y:=-4.8 yaw:=1.5708
dev_ws のシム (先に別端末で sim.launch.py world:=field_2027 drive:=omni を上げる):
    ros2 launch nr27_localization localization.launch.py sim:=true x:=-4.1 y:=-4.8 yaw:=1.5708

地図は maps/field_2027_<level>.yaml (level = ground / l1 / l2)。dev_ws の world から
scripts/import_dev_ws_maps.sh で取り込んだもの。座標も dev_ws と同じ (原点はフィールド中央、
赤が西 (-x)、北が +y。赤のスタートゾーンは (-4.1, -4.8) で北向き)。
出力: TF map -> odom、/amcl_pose。

sim:=true のときの違いは3つ: スキャンは /scan、時計は /clock (use_sim_time)、
odom -> base_link は Gazebo が出す。dev_ws の localization.launch.py (AMCL) とは同時に上げない
(map -> odom が2か所から出る)。

実機は odom -> base_link (オドメトリ) がまだ無い。fake_odom:=true で恒等の TF を流せば、
止まった状態で「スキャンが地図に重なるか」だけは確かめられる (動かすと追従しない)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

LEVELS = ('ground', 'l1', 'l2')


def arg(context, name):
    """起動引数の値を文字列で返す."""
    return LaunchConfiguration(name).perform(context)


def launch_setup(context):
    share = get_package_share_directory('nr27_localization')
    level = arg(context, 'level')
    if level not in LEVELS:
        raise RuntimeError(f'未知の level: {level} (使えるのは {LEVELS})')
    sim = arg(context, 'sim') == 'true'
    lidar = arg(context, 'lidar')
    scan_topic = '/scan' if sim else f'/{lidar}/scan'
    map_yaml = os.path.join(share, 'maps', f'field_2027_{level}.yaml')
    common = {'use_sim_time': sim}

    actions = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[common, {'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'amcl.yaml'),
                common,
                {
                    'scan_topic': scan_topic,
                    # '0' のような整数の表記でも float で渡す (int だと AMCL が型違いで落ちる)
                    'initial_pose.x': float(arg(context, 'x')),
                    'initial_pose.y': float(arg(context, 'y')),
                    'initial_pose.yaw': float(arg(context, 'yaw')),
                },
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[common, {'autostart': True, 'node_names': ['map_server', 'amcl']}],
        ),
    ]

    if arg(context, 'fake_odom') == 'true':
        if sim:
            raise RuntimeError('sim:=true では Gazebo が odom -> base_link を出すので fake_odom は使わない')
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_odom_tf',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
        ))
    if arg(context, 'launch_lidar') == 'true':
        if sim:
            raise RuntimeError('sim:=true ではスキャンは Gazebo が出すので launch_lidar は使わない')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nr27_sensors'), 'launch', 'lidar.launch.py')),
            launch_arguments={'lidars': lidar}.items(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        # dev_ws のシムと同じく、ROS の発見範囲をこの機械の中に絞る (端末で決めてあればそちら)。
        # 無線のアドレスが変わると、後から上げたノードがシムを見つけられなくなる (dev_ws の地雷表)
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', EnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE', default_value='LOCALHOST')),
        DeclareLaunchArgument('level', default_value='ground',
                              description='いる階: ground / l1 / l2 (階ごとに地図が違う)'),
        DeclareLaunchArgument('x', default_value='-4.1',
                              description='初期姿勢 [m]。既定は赤のスタートゾーン'),
        DeclareLaunchArgument('y', default_value='-4.8', description='初期姿勢 [m]'),
        DeclareLaunchArgument('yaw', default_value='1.5708',
                              description='初期姿勢 [rad]。既定は北向き (ランプへ向かう向き)'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='dev_ws の Gazebo とつなぐ (/scan・use_sim_time)'),
        DeclareLaunchArgument('lidar', default_value='ust30lx',
                              description='実機で使う LiDAR (nr27_sensors の名前)。/<lidar>/scan を使う'),
        DeclareLaunchArgument('fake_odom', default_value='false',
                              description='実機でオドメトリが無いとき、odom -> base_link を恒等で流す'),
        DeclareLaunchArgument('launch_lidar', default_value='false',
                              description='実機の LiDAR のドライバもここで上げる (nr27_sensors)'),
        OpaqueFunction(function=launch_setup),
    ])
