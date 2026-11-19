"""
独ステ1ユニットの試験用 launch.

使い方:
    # 別ターミナルで先に: ros2 run ros2can ros2can --nogui
    ros2 launch br_dokusute unit_test.launch.py

R1を押している間だけ動く。左スティックで平行移動、右スティック左右で旋回。
速さはスティックの倒し量に比例する (上限は br_joy_teleop_node.cpp の #define)。
起動したら車輪を手で前に揃えて
    ros2 service call /steer/set_zero std_srvs/srv/Trigger

つなぎ方:
    joy -> cmd_vel -> br_swerve_kinematics
        -> steer/angle_target -> br_steer_dm2325 -> steer/angle (戻る)
        -> drive/speed_target -> br_pid -> drive/duty -> br_drive_735
                                  ^-- drive/speed ------'
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('br_dokusute'), 'config', 'br_dokusute.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev', default_value='0', description='/dev/input/js番号'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # autorepeat_rate: 入力に変化が無くても/joyを送り直す周期[Hz]。
            # br_joy_teleopはjoyを受けたときだけcmd_velを送るので、0にすると
            # スティックを止めている間に下流がタイムアウトで止まる。
            parameters=[{
                'device_id': LaunchConfiguration('joy_dev'),
                'autorepeat_rate': 20.0,
                'deadzone': 0.05,
            }],
        ),
        Node(
            package='br_dokusute',
            executable='br_joy_teleop',
            name='br_joy_teleop',
            output='screen',
        ),
        Node(
            package='br_dokusute',
            executable='br_swerve_kinematics',
            name='br_swerve_kinematics',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='br_dokusute',
            executable='br_steer_dm2325',
            name='br_steer_dm2325',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='br_dokusute',
            executable='br_pid',
            name='br_pid',
            output='screen',
            parameters=[config],
            remappings=[
                ('setpoint', 'drive/speed_target'),
                ('measurement', 'drive/speed'),
                ('output', 'drive/duty'),
            ],
        ),
        Node(
            package='br_dokusute',
            executable='br_drive_735',
            name='br_drive_735',
            output='screen',
            parameters=[config],
        ),
    ])
