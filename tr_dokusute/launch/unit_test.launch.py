"""
独ステ1ユニットの試験用 launch.

使い方:
    # 別ターミナルで先に: ros2 run ros2can ros2can --nogui
    ros2 launch tr_dokusute unit_test.launch.py

R1を押している間だけ動く。左スティックで平行移動、右スティック左右で旋回。
速さはスティックの倒し量に比例する (上限は tr_joy_teleop_node.cpp の #define)。
起動したら車輪を手で前に揃えて
    ros2 service call /steer/set_zero std_srvs/srv/Trigger

つなぎ方:
    joy -> cmd_vel -> tr_swerve_kinematics
        -> steer/angle_target -> tr_steer_dm2325 -> steer/angle (戻る)
        -> drive/speed_target -> tr_pid -> drive/duty -> tr_drive_735
                                  ^-- drive/speed ------'
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('tr_dokusute'), 'config', 'tr_dokusute.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev', default_value='0', description='/dev/input/js番号'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # autorepeat_rate: 入力に変化が無くても/joyを送り直す周期[Hz]。
            # tr_joy_teleopはjoyを受けたときだけcmd_velを送るので、0にすると
            # スティックを止めている間に下流がタイムアウトで止まる。
            parameters=[{
                'device_id': LaunchConfiguration('joy_dev'),
                'autorepeat_rate': 20.0,
                'deadzone': 0.05,
            }],
        ),
        Node(
            package='tr_dokusute',
            executable='tr_joy_teleop',
            name='tr_joy_teleop',
            output='screen',
        ),
        Node(
            package='tr_dokusute',
            executable='tr_swerve_kinematics',
            name='tr_swerve_kinematics',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='tr_dokusute',
            executable='tr_steer_dm2325',
            name='tr_steer_dm2325',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='tr_dokusute',
            executable='tr_pid',
            name='tr_pid',
            output='screen',
            parameters=[config],
            remappings=[
                ('setpoint', 'drive/speed_target'),
                ('measurement', 'drive/speed'),
                ('output', 'drive/duty'),
            ],
        ),
        Node(
            package='tr_dokusute',
            executable='tr_drive_735',
            name='tr_drive_735',
            output='screen',
            parameters=[config],
        ),
    ])
