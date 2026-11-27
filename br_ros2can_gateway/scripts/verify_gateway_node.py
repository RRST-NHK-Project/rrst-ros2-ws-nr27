#!/usr/bin/env python3
"""
逆運動学ノード・仲介ノード・ros2can の模擬を Gazebo なしで結合試験する（SW-66）.

使い方:
  python3 br_ros2can_gateway/scripts/verify_gateway_node.py [--domain 5] [--keep-logs]
  ros2 run br_ros2can_gateway verify_gateway_node.py

構成:
  この試験 ─/cmd_vel─▶ swerve_drive ─/drive/module_cmd─▶ gateway_node ─serial_tx─▶ ros2can_fake
     └──/odom（拍子。Gazebo の代わり）───────────────────────────────────────────────▲
                                                                          └──▶ /cmd_vel_plant

検査項目:
  1. 指令した機体速度が模擬の車輪を通って戻ってくること（符号・単位・スロットの並び）
  2. 止めるふるまい（dev_ws の docs/control.md の「止めるふるまい」の全行・SW-71 機体入力）

ROS_DOMAIN_ID は走っているシムと重ならない番号にする（既定 5）。
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)

TESTS = []
_NOTE = []


def layer_scripts():
    """
    作戦の層（robocon2027_sim の mission_fsm.py）の置き場所を返す（無ければ None）.

    層は dev_ws（非公開）にあり、このパッケージだけを入れたワークスペースには無い。
    """
    cands = [os.path.join(HERE, '..', '..', 'robocon2027_sim', 'scripts')]
    try:
        from ament_index_python.packages import get_package_prefix
        cands.insert(0, os.path.join(
            get_package_prefix('robocon2027_sim'), 'lib', 'robocon2027_sim'))
    except Exception:                       # noqa: BLE001  ament に見つからない
        pass
    return next((os.path.realpath(c) for c in cands
                 if os.path.isfile(os.path.join(c, 'mission_fsm.py'))), None)


def case(name):
    """試験ケースを登録するデコレータ."""
    def deco(fn):
        TESTS.append((name, fn))
        return fn
    return deco


def note(text):
    """合格メッセージの補足を記録する."""
    _NOTE.append(text)


def check(cond, text):
    """条件が偽のとき AssertionError を送出する."""
    if not cond:
        raise AssertionError(text)


class Proc:
    """独立したセッションで起動・停止する子プロセス 1 つ分."""

    def __init__(self, name, script, args, logdir):
        self.name = name
        self.log = open(os.path.join(logdir, f'{name}.log'), 'ab')
        self.p = subprocess.Popen(
            [sys.executable, os.path.join(HERE, script), *args],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.pgid = self.p.pid          # start_new_session により PGID == PID

    def send(self, sig):
        """子プロセスグループにシグナル sig を送る."""
        if self.p.poll() is None and self.pgid > 1:
            os.killpg(self.pgid, sig)

    def stop(self, sig=signal.SIGINT, wait=3.0):
        """子プロセスグループを停止して終了コードを返す."""
        self.send(signal.SIGCONT)
        self.send(sig)
        try:
            self.p.wait(wait)
        except subprocess.TimeoutExpired:
            self.send(signal.SIGKILL)
            self.p.wait(2.0)
        self.log.close()
        return self.p.returncode


def main():
    """結合試験ベンチを起動し、全 25 項目を実行する."""
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        '--domain', type=int, default=5,
        help='ROS_DOMAIN_ID（走っているシムと重ならない番号）')
    ap.add_argument(
        '--keep-logs', action='store_true',
        help='子のプロセスのログを消さずに場所を出す')
    args = ap.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    # DDS の範囲は環境フックが設定した既定（CycloneDDS + LOCALHOST）を継承する
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = os.environ.get(
        'BR_DISCOVERY_RANGE', 'LOCALHOST')
    os.environ['PYTHONUNBUFFERED'] = '1'

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSPresetProfiles, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Empty, Float64MultiArray, Int16MultiArray, String
    from std_srvs.srv import Trigger
    import yaml

    import gateway_map as G

    m = G.load()
    ws_tmp = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'tmp'))
    logdir = tempfile.mkdtemp(
        prefix='verify_gateway_node_', dir=ws_tmp if os.path.isdir(ws_tmp) else None)
    procs = {}

    class Bench(Node):
        """結合試験の信号生成・受信監視ノード."""

        def __init__(self):
            super().__init__('verify_gateway_node')
            self.cmd = (0.0, 0.0, 0.0)
            self.t0 = time.monotonic()
            self.plant = None           # 模擬から戻ってきた機体速度 (vx, vy, ω)
            self.status = None
            self.flags_seen = set()
            self.tx = {}
            self.tx_zero_t = {}
            self.rx = {}
            self.truth = None
            self.odom = self.create_publisher(
                Odometry, '/odom', QoSPresetProfiles.SENSOR_DATA.value)
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.estop = self.create_publisher(Empty, '/mission_fsm/estop', 10)
            self.freeze = self.create_publisher(Bool, '/mission_fsm/freeze', 10)
            self.fault = self.create_publisher(Int16MultiArray, '/ros2can_fake/fault', 10)
            self.runaway = self.create_publisher(Float64MultiArray, '/ros2can_fake/runaway', 10)
            self.create_subscription(Twist, '/cmd_vel_plant', self.on_plant, 10)
            self.create_subscription(String, '/hw/status', self.on_status, 10)
            self.create_subscription(
                Float64MultiArray, '/ros2can_fake/state',
                lambda msg: setattr(self, 'truth', list(msg.data)), 10)
            for d in (m.dev_steer, m.dev_drive):
                self.create_subscription(
                    Int16MultiArray, f'/serial_tx_{d}',
                    lambda msg, d=d: self.on_tx(d, msg), 10)
                self.create_subscription(
                    Int16MultiArray, f'/serial_rx_{d}',
                    lambda msg, d=d: self.rx.__setitem__(d, list(msg.data)), 10)
            self.arm = self.create_client(Trigger, '/br_ros2can_gateway/arm')
            self.start_calls = 0
            self.create_service(Trigger, '/mission_fsm/start', self.on_layer_start)
            self.press = self.create_client(Trigger, '/ros2can_fake/press_start')
            self.hold_pub = self.create_publisher(Bool, '/ros2can_fake/start_switch', 10)
            self.team_sw = self.create_publisher(String, '/ros2can_fake/team_switch', 10)
            self.estop_btn = self.create_publisher(Bool, '/ros2can_fake/estop_button', 10)
            self.freeze_rx = self.create_publisher(Bool, '/ros2can_fake/freeze_receiver', 10)
            self.inputs_off = self.create_publisher(Bool, '/ros2can_fake/inputs_off', 10)
            self.layer_estop = 0
            self.layer_freeze = []
            self.team = None
            self.create_subscription(
                Empty, '/mission_fsm/estop',
                lambda _m: setattr(self, 'layer_estop', self.layer_estop + 1), 10)
            self.create_subscription(
                Bool, '/mission_fsm/freeze', lambda msg: self.layer_freeze.append(msg.data), 10)
            latched = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                String, '/hw/team', lambda msg: setattr(self, 'team', msg.data), latched)
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.02, self.send_odom, clock=steady)
            self.create_timer(0.05, self.send_cmd, clock=steady)

        def send_odom(self):
            t = time.monotonic() - self.t0
            msg = Odometry()
            msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1.0) * 1e9)
            self.odom.publish(msg)

        def send_cmd(self):
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = self.cmd
            self.cmd_pub.publish(msg)

        def on_plant(self, msg):
            self.plant = (msg.linear.x, msg.linear.y, msg.angular.z)

        def on_status(self, msg):
            st = json.loads(msg.data)
            if 'gw' in procs and st.get('pid') not in (None, procs['gw'].p.pid):
                return                  # 前項目の残存仲介ノードが出した古い状態は無視
            self.status = st
            self.flags_seen |= {f.split(':')[1] for f in self.status['flags']}

        def on_tx(self, dev, msg):
            self.tx[dev] = list(msg.data)
            if not any(msg.data):
                self.tx_zero_t[dev] = time.monotonic()

        def spin(self, sec):
            end = time.monotonic() + sec
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.01)

        def until(self, pred, timeout, what):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.01)
                if pred():
                    return
            raise AssertionError(
                f'{timeout} s 待っても {what}（状態 {self.status}・機体速度 {self.fmt()}）')

        def fmt(self):
            return 'なし' if self.plant is None else '(%.3f, %.3f, %.3f)' % self.plant

        def mode(self, mode, reason=None):
            s = self.status
            return (s is not None and s['mode'] == mode
                    and (reason is None or s['reason'] == reason))

        def moving(self, vx, vy, om, tol=0.02):
            p = self.plant
            return (p is not None and abs(p[0] - vx) < tol
                    and abs(p[1] - vy) < tol and abs(p[2] - om) < tol)

        def stopped(self):
            return self.moving(0.0, 0.0, 0.0, 1e-3)

        def tx_all_zero(self):
            return len(self.tx) == 2 and not any(any(f) for f in self.tx.values())

        def call_arm(self):
            check(self.arm.wait_for_service(timeout_sec=3.0), '~/arm のサービスが見つからない')
            fut = self.arm.call_async(Trigger.Request())
            self.until(fut.done, 3.0, '~/arm が返らない')
            return fut.result()

        def on_layer_start(self, _req, res):
            self.start_calls += 1
            res.success, res.message = True, '試験の層'
            return res

        def press_start(self):
            check(
                self.press.wait_for_service(timeout_sec=3.0),
                '模擬の ~/press_start が見つからない')
            fut = self.press.call_async(Trigger.Request())
            self.until(fut.done, 3.0, '~/press_start が返らない')

        def hold_start(self, on):
            msg = Bool()
            msg.data = on
            self.hold_pub.publish(msg)

        def press_estop(self):
            self.estop.publish(Empty())

        def set_freeze(self, on):
            msg = Bool()
            msg.data = on
            self.freeze.publish(msg)

        def put(self, pub, value):
            msg = String() if isinstance(value, str) else Bool()
            msg.data = value
            pub.publish(msg)

        def run_again(self, vx=0.3):
            self.cmd = (vx, 0.0, 0.0)
            res = self.call_arm()
            check(res.success, f'合図を断られた：{res.message}')
            self.until(lambda: self.mode('run') and self.moving(vx, 0.0, 0.0), 3.0, '走らない')

    def start(name, script, *a):
        if name in procs:
            procs.pop(name).stop()
        procs[name] = Proc(name, script, list(a), logdir)
        return procs[name]

    def stop_all():
        for name in list(procs):
            procs.pop(name).stop()

    rclpy.init()
    b = Bench()

    # ── 止めるふるまい・動き ──────────────────────────────────────────
    @case('起動直後は脱力のまま（合図を受けるまで動かない）')
    def _():
        start('fake', 'ros2can_fake.py')
        start('ik', 'swerve_drive.py')
        start('gw', 'gateway_node.py')
        b.cmd = (0.3, 0.0, 0.0)
        b.until(
            lambda: b.mode('relax') and b.plant is not None and b.rx
            and all(b.rx[d][i * 6] == G.ST_RELAX for d in b.rx for i in range(4)),
            8.0, '仲介ノードの状態が出ない')
        b.spin(1.0)
        check(
            b.mode('relax') and not b.status['armed'] and b.stopped() and b.tx_all_zero(),
            f'合図の前に動いた・0 以外を送った（{b.status}・{b.fmt()}・{b.tx}）')
        note(
            '/cmd_vel 0.3 m/s を1秒流しても、送るのは全部 0・機体速度 0'
            f"（理由 {b.status['reason']}）")

    @case('serial_tx_[ID] の送り手は仲介ノードだけ')
    def _():
        n = {d: b.count_publishers(f'/serial_tx_{d}') for d in (m.dev_steer, m.dev_drive)}
        check(all(v == 1 for v in n.values()), f'送り手の数 {n}')
        note(f'送り手の数 {n}')

    @case('合図（~/arm）で走る。前へ 0.3 m/s が車輪を通って戻る')
    def _():
        b.run_again(0.3)
        drive = b.tx[m.dev_drive]
        want = round(0.3 / m.wheel_radius * m.drive_gear / G.VEL_LSB)
        ok = all(
            drive[i * 6] == G.MODE_VEL and abs(drive[i * 6 + 1] - want) <= 1
            for i in range(4))
        check(ok, f'駆動の指令 {drive}（目標は {want}）')
        note(f'機体速度 {b.fmt()}、駆動の目標 {want}（0.01 rad/s）')

    @case('真横・その場回転・斜め（ステアの向きと符号）')
    def _():
        got = []
        for cmd in ((0.0, 0.3, 0.0), (0.0, 0.0, 0.8), (0.2, -0.2, 0.3), (-0.3, 0.0, 0.0)):
            b.cmd = cmd
            b.until(lambda: b.moving(*cmd, 0.005), 3.0, f'{cmd} にならない')
            got.append(b.fmt())
        note('指令どおり ' + '・'.join(got))

    @case('上限を超える指令は、逆運動学ノードが全輪を同じ割合で縮める')
    def _():
        b.cmd = (2.0, 0.0, 0.0)
        b.until(
            lambda: b.plant is not None and abs(b.plant[0] - 1.25) < 0.01,
            3.0, '1.25 m/s にならない')
        b.spin(0.3)
        check(not any(f.endswith(':clip') for f in b.status['flags']), f'仲介ノードで切った {b.status}')
        note(f'2.0 m/s の指令で {b.fmt()}（仲介ノードの切り捨ては掛からない）')
        b.cmd = (0.3, 0.0, 0.0)
        b.until(lambda: b.moving(0.3, 0.0, 0.0), 3.0, '0.3 m/s へ戻らない')

    @case('フリーズ：駆動は速度 0・ステアは今の位置で保ち、解けたら合図なしで走る')
    def _():
        b.set_freeze(True)
        t = time.monotonic()
        b.until(lambda: b.mode('hold', 'freeze') and b.stopped(), 1.0, '保たない')
        dt = time.monotonic() - t
        b.spin(0.3)
        drive, steer = b.tx[m.dev_drive], b.tx[m.dev_steer]
        check(all(drive[i * 6] == G.MODE_VEL and drive[i * 6 + 1] == 0 for i in range(4)),
              f'駆動 {drive}')
        check(all(steer[i * 6] == G.MODE_POS and steer[i * 6 + 2] > 0 for i in range(4)),
              f'ステア {steer}')
        check(all(b.rx[d][i * 6] == G.ST_ON for d in b.rx for i in range(4)), f'脱力した {b.rx}')
        b.set_freeze(False)
        b.until(lambda: b.mode('run') and b.moving(0.3, 0.0, 0.0), 2.0, '解けても走らない')
        note(f'止まるまで {dt * 1000:.0f} ms。モータは有効のまま（状態 1）')

    @case('非常停止：脱力し、離しても次の合図まで脱力。押している間は合図を断る')
    def _():
        b.press_estop()
        b.until(lambda: b.mode('relax', 'estop') and b.stopped() and b.tx_all_zero(), 1.0, '脱力しない')
        res = b.call_arm()
        check(not res.success, '押している間に合図を受けた')
        b.spin(1.6)
        check(
            b.mode('relax', 'estop') and b.stopped(),
            f'合図なしで戻った（{b.status}・{b.fmt()}）')
        b.run_again()
        note('離した後 0.6 s たっても脱力のまま。合図で走る')

    @case('上流（/drive/module_cmd）の途絶：0.3 s 以内に脱力し、戻っても次の合図まで脱力')
    def _():
        b.tx_zero_t.clear()
        t = time.monotonic()
        procs['ik'].send(signal.SIGSTOP)
        b.until(lambda: b.mode('relax', 'cmd_lost') and b.tx_all_zero(), 1.0, '脱力しない')
        dt = max(b.tx_zero_t.values()) - t
        check(dt < 0.3, f'脱力を送るまで {dt:.3f} s')
        procs['ik'].send(signal.SIGCONT)
        b.spin(1.0)
        check(
            b.mode('relax', 'cmd_lost') and b.stopped(),
            f'合図なしで戻った（{b.status}・{b.fmt()}）')
        b.run_again()
        note(f'逆運動学ノードを止めてから脱力を送るまで {dt * 1000:.0f} ms')

    @case('帰還（serial_rx）の途絶：脱力し、戻っても次の合図まで脱力')
    def _():
        b.tx_zero_t.clear()
        t = time.monotonic()
        procs['fake'].send(signal.SIGSTOP)
        b.until(lambda: b.mode('relax', 'fb_lost') and b.tx_all_zero(), 1.0, '脱力しない')
        dt = max(b.tx_zero_t.values()) - t
        procs['fake'].send(signal.SIGCONT)
        b.spin(1.0)
        check(
            b.status['mode'] == 'relax' and not b.status['armed'] and b.stopped(),
            f'合図なしで戻った（{b.status}・{b.fmt()}）')
        b.run_again()
        note(f'模擬を止めてから脱力を送るまで {dt * 1000:.0f} ms')

    @case('仲介ノードが固まる：模擬の ESP32 が途絶で止めてラッチ。戻っても次の合図まで脱力')
    def _():
        t = time.monotonic()
        procs['gw'].send(signal.SIGSTOP)
        b.until(b.stopped, 1.0, '車輪が止まらない')
        dt = time.monotonic() - t
        b.spin(0.2)
        check(
            all(b.rx[d][i * 6] == G.ST_LATCHED for d in b.rx for i in range(4)),
            f'ラッチしていない {b.rx}')
        procs['gw'].send(signal.SIGCONT)
        b.spin(1.0)
        check(
            b.status['mode'] == 'relax' and not b.status['armed'] and b.stopped(),
            f'合図なしで戻った（{b.status}・{b.fmt()}）')
        check(
            all(b.rx[d][i * 6] == G.ST_RELAX for d in b.rx for i in range(4)),
            f'ラッチが外れていない {b.rx}')
        b.run_again()
        note(f'固めてから車輪が止まるまで {dt * 1000:.0f} ms（模擬の途絶 0.3 s）')

    @case('モータの故障：脱力し、直っても次の合図まで脱力')
    def _():
        msg = Int16MultiArray()
        msg.data = [m.dev_drive, 1, 13]
        b.fault.publish(msg)
        b.until(lambda: b.mode('relax', 'fault:13') and b.stopped(), 1.0, '脱力しない')
        msg.data = [m.dev_drive, 1, 0]
        b.fault.publish(msg)
        b.spin(1.0)
        check(
            b.status['mode'] == 'relax' and not b.status['armed'] and b.stopped(),
            f'合図なしで戻った（{b.status}・{b.fmt()}）')
        b.run_again()
        note('駆動の1台を故障（状態 13）にした')

    def inject(speed, stays=0.0):
        msg = Float64MultiArray()
        msg.data = [float(m.dev_drive), 1.0, float(speed), float(stays)]
        b.runaway.publish(msg)

    @case('暴走（11/9 の形：指令と逆向きに全速、故障コードなし）：0.3 s 続いたら脱力し、合図まで止めたまま')
    def _():
        t = time.monotonic()
        inject(-40.0)
        b.until(lambda: b.mode('relax', 'runaway:1'), 1.5, '暴走を見つけない')
        dt = time.monotonic() - t
        check(0.25 <= dt <= 0.7, f'脱力するまで {dt:.3f} s')
        check(b.tx_all_zero() or b.until(b.tx_all_zero, 0.5, '脱力を送らない') is None,
              f'指令 {b.tx}')
        inject(0.0)
        b.spin(1.0)
        check(
            b.mode('relax') and not b.status['armed'] and b.stopped(),
            f'合図なしで戻った（{b.status}・{b.fmt()}）')
        b.run_again()
        note(f'注入してから脱力を送るまで {dt * 1000:.0f} ms。モータの状態は正常のまま（故障コードなし）')

    @case('脱力しても回り続けるとき（実機で未確認の最悪の場合）：車輪が回っている間は合図を受けない')
    def _():
        inject(-40.0, 1.0)
        b.until(lambda: b.mode('relax', 'runaway:1'), 1.5, '暴走を見つけない')
        b.spin(0.3)
        res = b.call_arm()
        check(not res.success, '回っているのに合図を受けた')
        inject(0.0)
        b.spin(0.5)
        b.run_again()
        note(f'断った理由：{res.message}')

    @case('ステアを同じ向きへ回し続けても、範囲（±135°）の中で車輪を反転して向きを作る。模擬は脱力しない')
    def _():
        b.flags_seen.clear()
        worst = 0.0
        for k in range(1, 10):
            a = math.radians(80.0 * k)
            b.cmd = (0.3 * math.cos(a), 0.3 * math.sin(a), 0.0)
            b.until(lambda: b.moving(*b.cmd), 3.0, f'{80 * k}° へ向かない')
            worst = max(worst, max(abs(v) for v in b.rx[m.dev_steer][2::6]) * G.INTERNAL_LSB)
            check(b.mode('run'), f'止まった {b.status}')
        if m.steer_pos_limit is None:
            note(f'範囲なしの表：巻き戻しなし、内部の位置は最大 {worst:.1f} rad')
        else:
            from ament_index_python.packages import get_package_share_directory
            swerve_yaml = os.path.join(
                get_package_share_directory('br_ros2can_gateway'), 'config', 'swerve.yaml')
            with open(swerve_yaml, encoding='utf-8') as f:
                ik_range = yaml.safe_load(f).get('steer_range')
            if ik_range is None:
                check(b.flags_seen & {'unwind', 'flip'}, f'範囲の印が出ない（見た印 {b.flags_seen}）')
            check(
                worst <= m.steer_pos_limit + 0.15,
                f'内部の位置が {worst:.1f} rad まで行った')
            note(
                f'80° × 9 回＝720°。印 {sorted(b.flags_seen)}、'
                f'内部の位置は最大 {worst:.1f} rad（範囲 ±{m.steer_pos_limit:g}）')
        b.cmd = (0.3, 0.0, 0.0)
        b.until(lambda: b.moving(0.3, 0.0, 0.0), 3.0, '前へ戻らない')

    @case('終わるとき（SIGINT）は脱力を送ってから終わる')
    def _():
        gw = procs.pop('gw')
        code = gw.stop(signal.SIGINT)
        b.spin(0.5)
        check(code == 0, f'終了コード {code}')
        check(b.tx_all_zero(), f'最後に送った指令が 0 でない {b.tx}')
        b.until(b.stopped, 1.0, '車輪が止まらない')
        note('最後の指令は全部 0')
        stop_all()

    # ── 起動の断り方・スイッチ・入力 ─────────────────────────────────
    @case('serial_tx_[ID] にほかの送り手がいたら起動しない')
    def _():
        other = b.create_publisher(Int16MultiArray, f'/serial_tx_{m.dev_steer}', 10)
        b.tx.clear()
        gw = Proc('gw_other', 'gateway_node.py', ['--discover', '3'], logdir)
        end = time.monotonic() + 8.0
        while gw.p.poll() is None and time.monotonic() < end:
            b.spin(0.1)
        code = gw.stop()
        b.destroy_publisher(other)
        check(code == 3, f'終了コード {code}（3 のはず）')
        check(not b.tx, f'断る前に送った {b.tx}')
        note('終了コード 3。断るまで何も送らない')

    @case('実機の形（--hw real）は、仮の値が残っている間は起動しない')
    def _():
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, 'gateway_node.py'), '--hw', 'real'],
            capture_output=True, text=True, timeout=20)
        check(
            r.returncode == 2 and '仮の値' in r.stderr,
            f'終了コード {r.returncode}：{r.stderr.strip()[-200:]}')
        note(r.stderr.strip().splitlines()[-1][:60] + '…')

    @case('--start-switch は --auto-arm と一緒に使えない。inputs.start が未定なら起動しない')
    def _():
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, 'gateway_node.py'),
             '--start-switch', '--auto-arm'],
            capture_output=True, text=True, timeout=20)
        check(
            r.returncode == 2 and '一緒に使えない' in r.stderr,
            f'終了コード {r.returncode}：{r.stderr.strip()[-200:]}')
        with open(G.DEFAULT_PATH, encoding='utf-8') as f:
            d = yaml.safe_load(f)
        d['inputs']['start'] = None
        path = os.path.join(logdir, 'hw_no_start.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(d, f, allow_unicode=True)
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, 'gateway_node.py'),
             '--start-switch', '--map', path],
            capture_output=True, text=True, timeout=20)
        check(
            r.returncode == 2 and 'inputs.start' in r.stderr,
            f'終了コード {r.returncode}：{r.stderr.strip()[-200:]}')
        note('どちらも終了コード 2')

    @case('--auto-arm は、上流が最初に動けと言ったときの1回だけ。タイヤが 30° 向いて起動しても、原点（--zeros）を渡せばまっすぐ進む')
    def _():
        b.plant = b.status = None
        zero = -m.steer_gear * math.radians(30.0)
        start('fake', 'ros2can_fake.py', '--steer-start', '30,30,30,30')
        start('ik', 'swerve_drive.py')
        start('gw', 'gateway_node.py', '--auto-arm', '--zeros=' + ','.join([f'{zero:.6f}'] * 4))
        b.cmd = (0.0, 0.0, 0.0)
        b.until(lambda: b.mode('relax') and b.plant is not None, 10.0, '仲介ノードの状態が出ない')
        b.spin(1.0)
        procs['fake'].send(signal.SIGSTOP)
        b.until(lambda: b.mode('relax', 'fb_lost'), 1.5, '帰還の途絶に気づかない')
        procs['fake'].send(signal.SIGCONT)
        b.spin(1.0)
        check(b.mode('relax') and not b.status['armed'], f'指令が 0 の間に動かした {b.status}')
        b.cmd = (0.3, 0.0, 0.0)
        b.until(lambda: b.mode('run') and b.moving(0.3, 0.0, 0.0), 5.0, '走らない')
        got = b.fmt()
        b.press_estop()
        b.until(lambda: b.mode('relax', 'estop'), 1.0, '脱力しない')
        b.spin(1.6)
        check(
            b.mode('relax', 'estop') and b.stopped(),
            f'止めた後に自分で戻った（{b.status}・{b.fmt()}）')
        note(
            f'指令が 0 の間は脱力（途中で帰還が途切れても使い切らない）。'
            f'動けの指令で {got}。非常停止の後は自分で戻らない')
        stop_all()

    @case('スタートスイッチ（--start-switch）：押すと動かして層の ~/start を1回呼ぶ。押したまま起動・非常停止の間は数えない・呼ばない')
    def _():
        b.plant = b.status = None
        b.start_calls = 0
        b.hold_start(False)
        start('fake', 'ros2can_fake.py')
        start('ik', 'swerve_drive.py')
        b.cmd = (0.3, 0.0, 0.0)
        check(b.press.wait_for_service(timeout_sec=8.0), '模擬が上がらない')
        start('gw', 'gateway_node.py', '--start-switch')
        b.until(
            lambda: (
                b.hold_start(True),
                b.status is not None and b.mode('relax')
                and b.status.get('start', {}).get('pressed') is True,
            )[1],
            8.0, 'スタートスイッチの読みが出ない')
        b.spin(1.0)
        check(
            b.mode('relax') and not b.status['armed'] and b.start_calls == 0 and b.stopped(),
            f'押したまま起動して動いた・層を呼んだ（{b.status}・呼んだ回数 {b.start_calls}）')
        b.hold_start(False)
        b.until(lambda: b.status['start']['pressed'] is False, 1.0, '離した読みにならない')
        b.spin(0.2)
        b.press_start()
        b.until(lambda: b.mode('run') and b.moving(0.3, 0.0, 0.0), 3.0, '押しても走らない')
        b.spin(0.5)
        check(
            b.start_calls == 1 and b.status['start']['count'] == 1,
            f"層を呼んだ回数 {b.start_calls}・数えた回数 {b.status['start']}")
        b.press_start()
        b.spin(0.5)
        check(
            b.mode('run') and b.start_calls == 1 and b.status['start']['count'] == 2,
            f'走っている間に押して層を呼んだ（{b.status}・呼んだ回数 {b.start_calls}）')
        b.cmd = (0.0, 0.0, 0.0)
        b.until(b.stopped, 2.0, '止まらない')
        b.hold_start(True)
        b.spin(1.0)
        b.hold_start(False)
        b.spin(0.3)
        check(b.start_calls == 2, f'押したまま 1 s で層を呼んだ回数 {b.start_calls - 1}（1 のはず）')
        b.press_estop()
        b.until(lambda: b.mode('relax', 'estop'), 1.0, '非常停止で脱力しない')
        b.press_start()
        b.spin(0.5)
        check(
            b.mode('relax', 'estop') and b.start_calls == 2 and b.stopped(),
            f'非常停止の間に押して動いた・層を呼んだ（{b.status}・呼んだ回数 {b.start_calls}）')
        b.cmd = (0.3, 0.0, 0.0)
        b.spin(1.0)
        check(b.mode('relax', 'estop') or b.mode('relax'), f'解けただけで動いた {b.status}')
        check(b.stopped(), f'解けただけで動いた {b.fmt()}')
        b.press_start()
        b.until(lambda: b.mode('run') and b.moving(0.3, 0.0, 0.0), 3.0, '解けた後に押しても走らない')
        b.spin(0.3)
        check(b.start_calls == 3, f'層を呼んだ回数 {b.start_calls}（3 のはず）')
        note(
            '押したまま起動 → 離すまで脱力・層は呼ばない。押す → 走る・層を1回。'
            '走っている間に押す → 断る・呼ばない。止まって押したまま 1 s → 1回。'
            '非常停止の間 → 断る・呼ばない。解けた後に押す → 走る'
            f"（数えた {b.status['start']['count']} 回、層 {b.start_calls} 回）")
        stop_all()

    def inputs_stack(*gw_args, yaml_path=None):
        """機体入力試験用の3ノードを起動して走らせる."""
        b.plant = b.status = None
        for pub in (b.estop_btn, b.freeze_rx, b.inputs_off):
            b.put(pub, False)
        start('fake', 'ros2can_fake.py', '--team', 'red')
        start('ik', 'swerve_drive.py')
        start('gw', 'gateway_node.py', *gw_args, *(['--map', yaml_path] if yaml_path else []))
        b.cmd = (0.3, 0.0, 0.0)
        b.until(lambda: b.mode('relax', 'latched'), 10.0, '新しい仲介ノードの状態が出ない')
        b.until(
            lambda: 'inputs' in b.status and b.status['inputs'].get('estop') is False
            and b.status['motor_status']
            and all(v in (0, 1) for v in b.status['motor_status']),
            5.0, '入力の読みが出ない・模擬のラッチが解けない')
        b.spin(0.3)
        if '--auto-arm' in gw_args:
            b.until(
                lambda: b.mode('run') and b.moving(0.3, 0.0, 0.0),
                5.0, '--auto-arm で走らない')
        else:
            b.run_again()

    def rules_yaml(name, **rules):
        with open(G.DEFAULT_PATH, encoding='utf-8') as f:
            d = yaml.safe_load(f)
        d['input_rules'].update(rules)
        path = os.path.join(logdir, name)
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(d, f, allow_unicode=True)
        return path

    @case('配線の非常停止のボタン（SW-71）：押すと脱力して層へ非常停止を出し、押している間は合図とスタートスイッチを断る。離しても戻らない')
    def _():
        b.start_calls = 0
        inputs_stack('--start-switch')
        b.layer_estop = 0
        t = time.monotonic()
        b.put(b.estop_btn, True)
        b.until(lambda: b.mode('relax', 'estop') and b.stopped(), 1.0, '脱力しない')
        dt = time.monotonic() - t
        b.spin(1.2)
        n = b.layer_estop
        check(n >= 2, f'押している間に層へ出した非常停止が {n} 通（0.5 s ごとに出し直すはず）')
        res = b.call_arm()
        check(not res.success and 'ボタン' in res.message, f'押している間に合図を受けた（{res.message}）')
        b.press_start()
        b.spin(0.6)
        check(
            b.mode('relax', 'estop') and b.start_calls == 0,
            f'押している間にスタートスイッチで動いた・層を呼んだ（{b.status}）')
        b.put(b.estop_btn, False)
        b.spin(1.5)
        check(
            b.mode('relax') and b.stopped() and b.status['inputs']['estop'] is False,
            f'離しただけで動いた {b.status}')
        b.press_start()
        b.until(
            lambda: b.mode('run') and b.moving(0.3, 0.0, 0.0),
            3.0, '離した後にスタートスイッチを押しても走らない')
        check(b.start_calls == 1, f'層を呼んだ回数 {b.start_calls}')
        note(
            f'押してから脱力まで {dt * 1000:.0f} ms・層へ {n} 通。'
            '押している間は合図・スタートスイッチを断る。'
            '離しても脱力のまま、スタートスイッチで走る')
        stop_all()

    @case('フリーズの受信機（SW-71）：止めろで層へ true を出して速度 0 で保ち、消えたら false を1回出して合図なしで走る')
    def _():
        inputs_stack('--auto-arm')
        b.layer_freeze.clear()
        b.spin(1.0)
        check(b.layer_freeze == [], f'受信機が止めろでないのに層へ出した {b.layer_freeze}')
        b.put(b.freeze_rx, True)
        b.until(lambda: b.mode('hold', 'freeze') and b.stopped(), 1.0, '保たない')
        b.spin(1.5)
        trues = [v for v in b.layer_freeze if v]
        check(
            len(trues) >= 2 and False not in b.layer_freeze,
            f'止めろの間に層へ出した値 {b.layer_freeze}')
        b.layer_freeze.clear()
        b.put(b.freeze_rx, False)
        b.until(lambda: b.mode('run') and b.moving(0.3, 0.0, 0.0), 2.0, '再開で走らない')
        b.spin(1.5)
        check(b.layer_freeze == [False], f'再開で層へ出した値 {b.layer_freeze}（false 1回のはず）')
        note(
            f'止めろの間に true を {len(trues)} 通（1 s ごと）、'
            'モータは有効のまま。消えたら false を1回、合図なしで走る')
        stop_all()

    @case('陣営スイッチ（SW-71）：/hw/team に red・blue を出し、入力の帰還が途切れたら空にする。層の read_team_switch が読む')
    def _():
        b.team = None
        inputs_stack('--auto-arm', yaml_path=rules_yaml('lost_ignore.yaml', lost='ignore'))
        b.until(lambda: b.team == 'red', 2.0, '/hw/team が red にならない')
        b.put(b.team_sw, 'blue')
        b.until(lambda: b.team == 'blue', 1.0, '/hw/team が blue にならない')
        got = [b.team]
        b.put(b.inputs_off, True)
        b.until(lambda: b.team == '', 1.0, '入力が途切れても /hw/team が空にならない')
        check(b.mode('run'), f'入力の途絶（lost: ignore）で走りを止めた {b.status}')
        b.put(b.inputs_off, False)
        b.until(lambda: b.team == 'blue', 1.0, '入力が戻っても /hw/team が戻らない')
        b.cmd = (0.0, 0.0, 0.0)
        b.until(b.stopped, 2.0, '止まらない')
        layer = layer_scripts()
        if layer is None:
            note(f'red → {got[0]} → 途切れて空 → 戻る。lost: ignore では走りを止めない。'
                 '層（robocon2027_sim）が無いので read_team_switch は確かめていない')
            stop_all()
            return
        code = (
            f'import sys; sys.path.insert(0, {layer!r}); '
            'import mission_fsm as mf; print(mf.read_team_switch() or "unknown")'
        )
        out = subprocess.run(
            [sys.executable, '-c', code], capture_output=True, text=True, timeout=30
        ).stdout.strip()
        check(out.endswith('blue'), f'層の read_team_switch が {out!r}（blue のはず）')
        b.put(b.inputs_off, True)
        b.until(lambda: b.team == '', 1.0, '/hw/team が空にならない')
        out2 = subprocess.run(
            [sys.executable, '-c', code], capture_output=True, text=True, timeout=30
        ).stdout.strip()
        check(out2.endswith('unknown'), f'入力が途切れているのに read_team_switch が {out2!r}（unknown のはず）')
        b.put(b.inputs_off, False)
        note(
            f'red → {got[0]} → 途切れて空 → 戻る。層の read_team_switch は '
            f'{out.split()[-1]}・途切れている間は {out2.split()[-1]}。lost: ignore では走りを止めない')
        stop_all()

    @case('入力の帰還の途絶（input_rules.lost）：hold は速度 0 で保って戻れば走る、relax は脱力して合図まで戻らない')
    def _():
        inputs_stack('--auto-arm', yaml_path=rules_yaml('lost_hold.yaml', lost='hold'))
        b.put(b.inputs_off, True)
        b.until(lambda: b.mode('hold', 'input_lost') and b.stopped(), 1.0, 'hold で保たない')
        b.put(b.inputs_off, False)
        b.until(lambda: b.mode('run') and b.moving(0.3, 0.0, 0.0), 2.0, 'hold で戻っても走らない')
        stop_all()
        inputs_stack('--auto-arm', yaml_path=rules_yaml('lost_relax.yaml', lost='relax'))
        b.put(b.inputs_off, True)
        b.until(lambda: b.mode('relax', 'input_lost') and b.stopped(), 1.0, 'relax で脱力しない')
        b.put(b.inputs_off, False)
        b.spin(1.0)
        check(b.mode('relax') and b.stopped(), f'relax で戻っただけで走った {b.status}')
        b.run_again()
        note('hold：途切れて保つ → 戻って走る。relax：途切れて脱力 → 戻っても脱力 → 合図で走る')
        stop_all()

    @case('割り当て表が機体と食い違うと、動きに出る（模擬の側だけ駆動1台の向きを逆にする）')
    def _():
        with open(G.DEFAULT_PATH, encoding='utf-8') as f:
            d = yaml.safe_load(f)
        d['motors']['drive'][0]['sign'] = -1
        path = os.path.join(logdir, 'hw_flipped.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(d, f, allow_unicode=True)
        b.plant = b.status = None
        start('fake', 'ros2can_fake.py', '--hw-yaml', path)
        start('ik', 'swerve_drive.py')
        start('gw', 'gateway_node.py', '--auto-arm')
        b.cmd = (0.3, 0.0, 0.0)
        b.until(
            lambda: b.mode('run') and b.plant is not None
            and any(abs(v) > 0.01 for v in b.plant),
            10.0, '走らない')
        b.spin(1.0)
        check(not b.moving(0.3, 0.0, 0.0, 0.05), f'食い違いが動きに出ない {b.fmt()}')
        note(f'前へ 0.3 m/s の指令で {b.fmt()}（合っていれば (0.300, 0.000, 0.000)）')
        stop_all()

    bad = 0
    try:
        for name, fn in TESTS:
            _NOTE.clear()
            try:
                fn()
                print(f"  OK   {name}：{'／'.join(_NOTE)}", flush=True)
            except Exception as e:                                 # noqa: BLE001
                bad += 1
                print(f'  NG   {name}：{type(e).__name__} {e}', flush=True)
    finally:
        stop_all()
        b.destroy_node()
        rclpy.try_shutdown()
    print()
    print(f"{'合格' if not bad else '不合格'} {len(TESTS)} 項目 / 指摘 {bad} 件")
    if bad or args.keep_logs:
        print(f'子のプロセスのログ：{logdir}')
    else:
        for f in os.listdir(logdir):
            os.remove(os.path.join(logdir, f))
        os.rmdir(logdir)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
