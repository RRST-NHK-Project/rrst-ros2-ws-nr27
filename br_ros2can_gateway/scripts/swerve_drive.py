#!/usr/bin/env python3
"""
逆運動学ノード（SW-66、dev_ws の決定事項58・60）.

機体速度 `/cmd_vel` を4輪のステア角・車輪周速指令へ変換し、実時間周期で配信し続ける。
仲介ノード（`gateway_node.py`）が 0.2 s 途絶で脱力するため、速度 0 の間も一定周期で送る。

入出力:
  sub  /cmd_vel           (geometry_msgs/Twist, [m/s], [rad/s])
  sub  /drive/module_state (std_msgs/Float64MultiArray, [δ0..δ3[rad], s0..s3[m/s]])
  pub  /drive/module_cmd  (std_msgs/Float64MultiArray, [δ0..δ3[rad], s0..s3[m/s]])

使い方:
  ros2 run br_ros2can_gateway swerve_drive.py
  ros2 run br_ros2can_gateway swerve_drive.py --rate 50.0 --cmd-timeout 0.5
"""

import argparse
import math
import os
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def load_swerve():
    """同じパッケージの swerve.py（足回りの式）を読み込む."""
    here = os.path.dirname(os.path.realpath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    import swerve
    return swerve


def module_command(S, p, cmd, delta_now):
    """
    機体速度指令 (vx, vy, ω) と現在ステア角から (目標ステア角列 [rad], 車輪周速列 [m/s]) を返す.

    `swerve.ik_command` の後、周速上限 `wheel_speed_max` を超える輪があれば
    全輪を同じ比率で縮小する（1輪だけ切ると進行方向が曲がるため、`swerve.plant_step` と同じ扱い）。
    """
    rows = S.ik_command(cmd[0], cmd[1], cmd[2], delta_now, p)
    deltas = [float(t) for t, _, _ in rows]
    speeds = [float(s) for _, s, _ in rows]
    top = max(abs(s) for s in speeds)
    if p.wheel_speed_max > 0 and top > p.wheel_speed_max:
        scale = p.wheel_speed_max / top
        speeds = [s * scale for s in speeds]
    return deltas, speeds


class SwerveDrive(Node):
    """/cmd_vel を4輪モジュール指令 /drive/module_cmd へ定周期変換するノード."""

    def __init__(self, args, S, p):
        super().__init__('swerve_drive')
        self.args, self.S, self.p = args, S, p
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = None                     # 最後に /cmd_vel を受けた時刻（monotonic [s]）
        self.state = None                     # /drive/module_state のステア角 [rad]
        self.state_t = None
        self.last = [0.0] * p.n               # 最後に出したステア角目標 [rad]
        self.stale = False

        self.pub = self.create_publisher(Float64MultiArray, args.out, 10)
        self.create_subscription(Twist, args.inp, self.on_cmd, 10)
        self.create_subscription(Float64MultiArray, args.state, self.on_state, 10)
        # 実機と同じ実時間タイマ（/clock 駆動だとシムで CPU 負荷が高くなる）
        self.create_timer(
            1.0 / args.rate, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            f'逆運動学：{args.inp} -> {args.out}（{args.rate:g} Hz、モジュール {p.n} 個、'
            f'車輪の周速の上限 {p.wheel_speed_max} m/s、'
            f'/cmd_vel が {args.cmd_timeout} s 来なければ速さ 0）')

    def on_cmd(self, msg):
        """機体速度指令を保持する（非有限値は無視）."""
        v = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(x) for x in v):
            self.cmd, self.cmd_t = v, time.monotonic()

    def on_state(self, msg):
        """帰還から復号された実ステア角を取り込む."""
        if len(msg.data) == 2 * self.p.n and all(math.isfinite(x) for x in msg.data):
            self.state, self.state_t = list(msg.data[:self.p.n]), time.monotonic()

    def tick(self):
        """定周期で逆運動学を解き /drive/module_cmd を送る."""
        now = time.monotonic()
        cmd = self.cmd
        late = self.cmd_t is None or now - self.cmd_t > self.args.cmd_timeout
        if late:
            if not self.stale and self.cmd_t is not None and any(cmd):
                self.get_logger().warn(
                    f'/cmd_vel が {self.args.cmd_timeout} s 来ないので速さ 0 を出す')
            cmd = (0.0, 0.0, 0.0)
        self.stale = late

        fresh = self.state_t is not None and now - self.state_t <= self.args.state_timeout
        delta_now = self.state if fresh else self.last
        deltas, speeds = module_command(self.S, self.p, cmd, delta_now)
        self.last = deltas

        out = Float64MultiArray()
        out.data = deltas + speeds
        self.pub.publish(out)


def main():
    """逆運動学ノードを起動する."""
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        '--rate', type=float, default=50.0,
        help='出す周期 [Hz]（実時間）。仲介ノードの rate_hz に揃える')
    ap.add_argument(
        '--cmd-timeout', type=float, default=0.5,
        help='/cmd_vel がこの秒数来なければ速さ 0 を出す [s]')
    ap.add_argument(
        '--state-timeout', type=float, default=0.5,
        help='/drive/module_state がこの秒数来なければ、最後に出した目標を今のステア角とみなす [s]')
    ap.add_argument(
        '--params', default=None,
        help='swerve.yaml（既定はこのパッケージの config/swerve.yaml）')
    ap.add_argument('--inp', default='/cmd_vel')
    ap.add_argument('--out', default='/drive/module_cmd')
    ap.add_argument('--state', default='/drive/module_state')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    S = load_swerve()
    p = S.load(args.params)
    rclpy.init()
    node = SwerveDrive(args, S, p)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
