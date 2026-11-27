#!/usr/bin/env python3
"""
ros2can と Damiao モータ／入力 ESP32 の模擬ノード（SW-66・SW-71、シム専用）.

`hw:=fake` で仲介ノードの通信相手となる。仲介ノード側の換算関数（gateway_map.py）を使わず、
ros2can ファーム（damiao.cpp / can_host プロファイル）の仕様から独立に実装している。

入出力:
  sub  /serial_tx_<dev>   (std_msgs/Int16MultiArray, 24要素: ステア/駆動 TX)
  sub  /odom              (nav_msgs/Odometry, シム時刻の拍子)
  sub  ~/fault            (std_msgs/Int16MultiArray, [dev, idx, code])
  sub  ~/runaway          (std_msgs/Float64MultiArray, [dev, idx, vel, stays])
  sub  ~/start_switch     (std_msgs/Bool) / ~/estop_button / ~/freeze_receiver / ~/inputs_off
  sub  ~/team_switch      (std_msgs/String, 'red'/'blue')
  pub  /serial_rx_<dev>   (std_msgs/Int16MultiArray, 24要素: 帰還 200 Hz 実時間)
  pub  /cmd_vel_plant     (geometry_msgs/Twist, 順運動学で戻した機体速度 [m/s], [rad/s])
  pub  ~/state            (std_msgs/Float64MultiArray, 真値 [δ0..δ3[rad], s0..s3[m/s]])
  srv  ~/press_start      (std_srvs/Trigger, --press-time 秒だけ押下)

時計は `/odom` のシム時刻から進め、帰還フレームは実時間 `--rx-rate` Hz で配信する。
"""

import argparse
from dataclasses import dataclass, field
import math
import os
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Bool, Float64MultiArray, Int16MultiArray, String
from std_srvs.srv import Trigger
import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from swerve_drive import load_swerve                               # noqa: E402, I100

N_SLOTS, PER_MOTOR, N_MOTORS = 24, 6, 4
MODE_RELAX, MODE_VEL, MODE_POS = 0, 1, 2
ST_RELAX, ST_ON, ST_LATCHED = 0, 1, 15
POS_LSB, VEL_LSB, INTERNAL_LSB = 0.001, 0.01, 0.1
INT16_MIN, INT16_MAX = -32768, 32767
DEFAULT_HW = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'gateway_map.yaml'))


def clamp(x, lo, hi):
    """値 x を [lo, hi] に制限する."""
    return min(max(x, lo), hi)


def int16(x):
    """値 x を丸めて signed 16-bit 整数に収める."""
    return int(clamp(round(x), INT16_MIN, INT16_MAX))


# ── ESP32・機体モデル（ROS 非依存） ──────────────────────────────────
@dataclass
class EspCfg:
    """ファーム（config.hpp）と ESC の特性パラメータ."""

    link_timeout: float = 0.3   # DAMIAO_CMD_TIMEOUT_MS（PC 指令途絶の許容時間）[s]
    vel_max: float = 45.6       # DAMIAO_MAX_VEL_RADPS [rad/s]
    p_max: float = 12.5         # DAMIAO_P_MAX [rad]（帰還位置折り返し・動作限界）
    pos_cmd_max: float = 12.0   # 位置指令を丸める範囲 [rad]
    run_max: float = 12.5       # 位置制御中にこれを超えたら脱力 [rad]
    enable_delay: float = 0.03  # 脱力から有効化までの待ち時間 [s]
    vel_lag: float = 0.0        # 速度制御の一次遅れ時定数 [s]（0 で瞬時）
    vel_accel: float = 0.0      # 速度制御の加速度上限 [rad/s^2]（0 で制限なし）
    coast: float = 0.0          # 脱力時の惰性減衰時定数 [s]（0 で即停止）


@dataclass
class Motor:
    """模擬する Damiao モータ 1 台分の状態."""

    pos: float = 0.0            # 連続回転位置 [rad]（電源投入位置が 0）
    vel: float = 0.0            # 出力軸角速度 [rad/s]
    mode: int = MODE_RELAX
    latched: bool = True
    pending: int = MODE_RELAX
    ready_at: float = 0.0
    count: int = 0
    fault: int = 0              # 非ゼロで故障状態コード
    runaway: float = 0.0        # 非ゼロで注入暴走速度 [rad/s]
    runaway_stays: bool = False


@dataclass
class Esp32:
    """ESP32 1 台（モータ 4 つ）の状態遷移と帰還生成."""

    cfg: EspCfg = field(default_factory=EspCfg)
    motors: list = field(default_factory=lambda: [Motor() for _ in range(N_MOTORS)])
    cmd: list = field(default_factory=lambda: [0] * N_SLOTS)
    last_tx: object = None      # 最後に TX フレームを受信した時刻 [s]

    def on_tx(self, frame, t):
        """TX フレーム（24要素）を時刻 t [s] に取り込む."""
        if len(frame) != N_SLOTS:
            return
        self.cmd, self.last_tx = [int(v) for v in frame], t

    def step(self, t, dt):
        """時刻 t [s] で dt [s] 分モータ状態を更新する."""
        c = self.cfg
        link_lost = self.last_tx is None or t - self.last_tx > c.link_timeout
        for i, mot in enumerate(self.motors):
            o = i * PER_MOTOR
            pc_mode = self.cmd[o]
            if link_lost or mot.fault:
                mot.latched = True
            elif pc_mode == MODE_RELAX:
                mot.latched = False
            mode = MODE_RELAX if mot.latched else pc_mode
            if mode not in (MODE_RELAX, MODE_VEL, MODE_POS):
                mode = MODE_RELAX
            vel_limit = self.cmd[o + 2] * VEL_LSB
            # 有効化前は ±12 rad、動作中は ±12.5 rad（2026-11-18 BLDC 仕様）
            pos_max = c.run_max if mot.mode == MODE_POS else c.pos_cmd_max
            if mode == MODE_POS and (abs(mot.pos) > pos_max or vel_limit <= 0.0):
                mode = MODE_RELAX
            if mode == MODE_RELAX:
                mot.mode = mot.pending = MODE_RELAX
            elif mode != mot.mode:
                if mot.pending != mode:
                    mot.pending, mot.ready_at = mode, t + c.enable_delay
                if t >= mot.ready_at:
                    mot.mode = mode
            if mot.mode == MODE_VEL:
                tgt = clamp(self.cmd[o + 1] * VEL_LSB, -c.vel_max, c.vel_max)
                decay = math.exp(-dt / c.vel_lag) if c.vel_lag > 0 else 0.0
                nxt = tgt + (mot.vel - tgt) * decay
                if c.vel_accel > 0:
                    nxt = mot.vel + clamp(nxt - mot.vel, -c.vel_accel * dt, c.vel_accel * dt)
                mot.vel = nxt
                mot.pos += mot.vel * dt
            elif mot.mode == MODE_POS:
                tgt = clamp(self.cmd[o + 1] * POS_LSB, -c.pos_cmd_max, c.pos_cmd_max)
                move = clamp(tgt - mot.pos, -vel_limit * dt, vel_limit * dt)
                mot.pos += move
                mot.vel = move / dt if dt > 0 else 0.0
            else:
                mot.vel *= math.exp(-dt / c.coast) if c.coast > 0 else 0.0
                mot.pos += mot.vel * dt
            if mot.runaway and (mot.mode == MODE_VEL or mot.runaway_stays):
                mot.vel = mot.runaway
            mot.count = (mot.count + 1) % (INT16_MAX + 1)

    def frame(self):
        """現在状態の RX 24 スロットを生成する."""
        out = [0] * N_SLOTS
        p = self.cfg.p_max
        for i, mot in enumerate(self.motors):
            if mot.fault:
                status = mot.fault
            elif mot.latched:
                status = ST_LATCHED
            else:
                status = ST_ON if mot.mode != MODE_RELAX else ST_RELAX
            wrapped = (mot.pos + p) % (2.0 * p) - p
            out[i * PER_MOTOR:(i + 1) * PER_MOTOR] = [
                status, mot.count, int16(mot.pos / INTERNAL_LSB),
                int16(wrapped / POS_LSB), int16(mot.vel / VEL_LSB), 0,
            ]
        return out


@dataclass(frozen=True)
class Hw:
    """模擬対象のハードウェア構成（--hw-yaml から読み込む）."""

    wheel_radius: float
    drive_gear: float
    steer_gear: float
    dev_steer: int
    dev_drive: int
    steer: tuple        # (module, index, sign) を module 順に並べたタプル
    drive: tuple
    dev_inputs: int = 0
    start_sw: tuple = None
    team_sw: tuple = None
    estop_sw: tuple = None
    freeze_sw: tuple = None
    team_on: str = 'blue'
    estop_on: int = 0
    freeze_on: int = 1

    def has_inputs(self):
        """いずれかの機体スイッチ入力が定義されているか返す."""
        return any(
            x is not None for x in (self.start_sw, self.team_sw, self.estop_sw, self.freeze_sw))


def load_hw(path=None):
    """gateway_map 互換 YAML からハードウェア配線・スイッチ定義を読み込む."""
    with open(path or DEFAULT_HW, encoding='utf-8') as f:
        d = yaml.safe_load(f)

    def rows(role):
        return tuple(sorted(
            (int(r['module']), int(r['index']), int(r['sign'])) for r in d['motors'][role]))

    def sw(name):
        v = d['inputs'].get(name)
        return None if v is None else tuple(int(x) for x in v)

    rules = d.get('input_rules') or {}
    return Hw(
        wheel_radius=float(d['wheel_radius']), drive_gear=float(d['drive']['gear']),
        steer_gear=float(d['steer']['gear']), dev_steer=int(d['devices']['steer']),
        dev_drive=int(d['devices']['drive']), steer=rows('steer'), drive=rows('drive'),
        dev_inputs=int(d['devices']['inputs']), start_sw=sw('start'), team_sw=sw('team'),
        estop_sw=sw('estop'), freeze_sw=sw('freeze'), team_on=rules.get('team_on', 'blue'),
        estop_on=int(rules.get('estop_on', 0)), freeze_on=int(rules.get('freeze_on', 1)),
    )


def input_frame(hw, start_pressed, team='red', estop=False, freeze=False):
    """入力用 ESP32 の 24 スロット配列を生成する（CAN ホストプロファイル固定並び）."""
    out = [0] * N_SLOTS

    def put(where, value):
        if where is not None:
            node, k = where
            out[(node - 1) * 5 + (k - 1)] = int(value)

    put(hw.start_sw, bool(start_pressed))
    put(hw.team_sw, team == hw.team_on)
    put(hw.estop_sw, hw.estop_on if estop else 1 - hw.estop_on)
    put(hw.freeze_sw, hw.freeze_on if freeze else 1 - hw.freeze_on)
    return out


def wheel_state(hw, steer_esp, drive_esp, start):
    """現在モータ状態から (ステア角 [rad], 車輪周速 [m/s]) を返す."""
    deltas = [
        math.atan2(math.sin(a), math.cos(a))
        for a in (
            s0 + sign * steer_esp.motors[idx].pos / hw.steer_gear
            for (_, idx, sign), s0 in zip(hw.steer, start)
        )
    ]
    speeds = [
        sign * drive_esp.motors[idx].vel / hw.drive_gear * hw.wheel_radius
        for _, idx, sign in hw.drive
    ]
    return deltas, speeds


# ── ROS 2 ノード ───────────────────────────────────────────────────────
class Ros2CanFake(Node):
    """ros2can・ESP32・Damiao モータの挙動を模擬する ROS 2 ノード."""

    def __init__(self, args, hw, cfg, swerve_mod, swerve_params, start):
        super().__init__('ros2can_fake')
        self.args = args
        self.hw = hw
        self.S = swerve_mod
        self.p = swerve_params
        self.start = start

        self.esp = {hw.dev_steer: Esp32(cfg=cfg), hw.dev_drive: Esp32(cfg=cfg)}
        self.t = None               # 最新の /odom シム時刻 [s]
        self.prev = None
        self.queue = []             # (到達時刻 [s], device_id, frame)

        self.rx = {
            d: self.create_publisher(Int16MultiArray, f'/serial_rx_{d}', 10) for d in self.esp
        }
        self.pub = self.create_publisher(Twist, args.out, 10)
        self.state = self.create_publisher(Float64MultiArray, '~/state', 10)
        for d in self.esp:
            self.create_subscription(
                Int16MultiArray, f'/serial_tx_{d}', lambda m, d=d: self.on_tx(d, m), 10)
        self.create_subscription(Int16MultiArray, '~/fault', self.on_fault, 10)
        self.create_subscription(Float64MultiArray, '~/runaway', self.on_runaway, 10)
        self.create_subscription(
            Odometry, args.clock_from, self.on_odom, QoSPresetProfiles.SENSOR_DATA.value)

        # 入力用 ESP32 の模擬（SW-71）
        self.start_hold = False
        self.start_until = 0.0
        self.team = args.team
        self.estop_btn = False
        self.freeze_rx = False
        self.inputs_off = False
        self.in_pub = None
        if hw.has_inputs():
            self.in_pub = self.create_publisher(
                Int16MultiArray, f'/serial_rx_{hw.dev_inputs}', 10)
            self.create_subscription(
                Bool, '~/start_switch', lambda m: setattr(self, 'start_hold', bool(m.data)), 10)
            self.create_service(Trigger, '~/press_start', self.on_press_start)
            self.create_subscription(String, '~/team_switch', self.on_team, 10)
            self.create_subscription(
                Bool, '~/estop_button', lambda m: setattr(self, 'estop_btn', bool(m.data)), 10)
            self.create_subscription(
                Bool, '~/freeze_receiver', lambda m: setattr(self, 'freeze_rx', bool(m.data)), 10)
            self.create_subscription(
                Bool, '~/inputs_off', lambda m: setattr(self, 'inputs_off', bool(m.data)), 10)

        self.create_timer(
            1.0 / args.rx_rate, self.send_rx, clock=Clock(clock_type=ClockType.STEADY_TIME))

        delay_msg = f'・指令の遅れ {args.tx_delay} s' if args.tx_delay > 0 else ''
        in_msg = (
            f'。機体の入力 serial_rx_{hw.dev_inputs}（陣営スイッチは {args.team}）'
            if hw.has_inputs() else ''
        )
        self.get_logger().info(
            f'ros2can の模擬：serial_tx_{hw.dev_steer}（ステア）・'
            f'serial_tx_{hw.dev_drive}（駆動）-> {args.out}。'
            f'指令が {cfg.link_timeout} s 来なければ脱力・ラッチ。'
            f'帰還 {args.rx_rate:g} Hz{delay_msg}{in_msg}')

    def on_team(self, msg):
        """模擬陣営スイッチの値を切り替える."""
        if msg.data in ('red', 'blue'):
            self.team = msg.data

    def on_press_start(self, _req, res):
        """模擬スタートスイッチを press_time 秒だけ押下状態にする."""
        self.start_until = time.monotonic() + self.args.press_time
        res.success = True
        res.message = f'スタートスイッチを {self.args.press_time:g} s 押す'
        return res

    def on_tx(self, dev, msg):
        """PC からの TX フレームを必要に応じて遅延キューに積む."""
        if self.t is None:
            return
        if self.args.tx_delay > 0:
            self.queue.append((self.t + self.args.tx_delay, dev, list(msg.data)))
        else:
            self.esp[dev].on_tx(msg.data, self.t)

    def on_fault(self, msg):
        """試験用故障注入：[デバイス ID, モータ番号, 故障コード（0 で解除）]."""
        if len(msg.data) == 3 and msg.data[0] in self.esp and 0 <= msg.data[1] < N_MOTORS:
            self.esp[msg.data[0]].motors[msg.data[1]].fault = int(msg.data[2])

    def on_runaway(self, msg):
        """試験用暴走注入：[デバイス ID, モータ番号, 速度 [rad/s], 脱力無視 (0/1)]."""
        ok = len(msg.data) == 4 and int(msg.data[0]) in self.esp
        if ok and 0 <= int(msg.data[1]) < N_MOTORS:
            mot = self.esp[int(msg.data[0])].motors[int(msg.data[1])]
            mot.runaway, mot.runaway_stays = float(msg.data[2]), bool(msg.data[3])

    def on_odom(self, msg):
        """/odom のシム時刻でモータを1歩進め、順運動学で機体速度を出す."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.t = t
        if not rclpy.ok():
            return
        if self.prev is None:
            self.prev = t
            return
        dt = min(max(t - self.prev, 0.0), 0.1)     # 時刻飛び時も 1 ステップ最大 0.1 s
        self.prev = t
        if dt <= 0.0:
            return
        while self.queue and self.queue[0][0] <= t:
            _, dev, frame = self.queue.pop(0)
            self.esp[dev].on_tx(frame, t)
        for esp in self.esp.values():
            esp.step(t, dt)
        deltas, speeds = wheel_state(
            self.hw, self.esp[self.hw.dev_steer], self.esp[self.hw.dev_drive], self.start)
        vbx, vby, om, _ = self.S.fk(speeds, deltas, self.p)
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = vbx, vby, om
        self.pub.publish(out)
        st = Float64MultiArray()
        st.data = [float(v) for v in deltas + speeds]
        self.state.publish(st)

    def send_rx(self):
        """実時間タイマでモータと入力用 ESP32 の帰還フレームを送る."""
        for d, esp in self.esp.items():
            out = Int16MultiArray()
            out.data = esp.frame()
            self.rx[d].publish(out)
        if self.in_pub is not None and not self.inputs_off:
            out = Int16MultiArray()
            out.data = input_frame(
                self.hw, self.start_hold or time.monotonic() < self.start_until,
                self.team, self.estop_btn, self.freeze_rx)
            self.in_pub.publish(out)


def main():
    """引数を解析し、ros2can 模擬ノードを起動する."""
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        '--hw-yaml', default=None,
        help='機体の作り（既定は config/gateway_map.yaml）')
    ap.add_argument(
        '--params', default=None,
        help='swerve.yaml（モジュールの位置。既定はこのパッケージの config）')
    ap.add_argument(
        '--link-timeout', type=float, default=EspCfg.link_timeout,
        help='指令が途切れたら脱力・ラッチ [s]')
    ap.add_argument(
        '--enable-delay', type=float, default=EspCfg.enable_delay,
        help='脱力から有効までの時間 [s]')
    ap.add_argument(
        '--vel-lag', type=float, default=0.0,
        help='駆動の速度の一次遅れの時定数 [s]。0 で瞬時')
    ap.add_argument(
        '--vel-accel', type=float, default=0.0,
        help='駆動の加速度の上限 [rad/s^2]（出力軸）。0 で上限なし')
    ap.add_argument(
        '--coast', type=float, default=0.0,
        help='脱力した車輪の速度が抜ける時定数 [s]。0 ですぐ止まる')
    ap.add_argument(
        '--pos-cmd-max', type=float, default=EspCfg.pos_cmd_max,
        help='角度の指令を丸める範囲 [rad]。ファームのいまの値は 12')
    ap.add_argument(
        '--pos-run-max', type=float, default=EspCfg.run_max,
        help='角度制御の間に内部の位置がこれを超えたら脱力 [rad]。ファームのいまの値は 12.5。'
             '「範囲を外せたら」を試すときは、この2つを広げ、割り当て表の '
             'steer.pos_limit を null にする')
    ap.add_argument(
        '--tx-delay', type=float, default=0.0,
        help='PC 側（ros2can）で指令が遅れる時間 [s]。0 で遅れなし')
    ap.add_argument(
        '--steer-start', default=None,
        help='電源を入れたときのステア角 [deg]（モジュール番号の順、カンマ区切り）。'
             '既定は全部 0＝前向き')
    ap.add_argument(
        '--rx-rate', type=float, default=200.0,
        help='帰還を出す周期 [Hz]（実時間）')
    ap.add_argument(
        '--press-time', type=float, default=0.3,
        help='~/press_start でスタートスイッチを押す時間 [s]（実時間）')
    ap.add_argument(
        '--team', choices=('red', 'blue'), default='red',
        help='陣営スイッチの起動の位置')
    ap.add_argument(
        '--clock-from', default='/odom',
        help='時刻（シム時間）と拍子を取るトピック（Odometry）')
    ap.add_argument('--out', default='/cmd_vel_plant')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    hw = load_hw(args.hw_yaml)
    S = load_swerve()
    p = S.load(args.params)
    start = (
        [math.radians(float(x)) for x in args.steer_start.split(',')]
        if args.steer_start else [0.0] * len(hw.steer)
    )
    if len(start) != len(hw.steer) or len(hw.steer) != p.n:
        print(
            f'模擬：モジュールの数が合わない（機体の作り {len(hw.steer)}・'
            f'swerve.yaml {p.n}・--steer-start {len(start)}）',
            file=sys.stderr)
        return 2

    cfg = EspCfg(
        link_timeout=args.link_timeout, enable_delay=args.enable_delay,
        vel_lag=args.vel_lag, vel_accel=args.vel_accel, coast=args.coast,
        pos_cmd_max=args.pos_cmd_max, run_max=args.pos_run_max,
    )
    rclpy.init()
    node = Ros2CanFake(args, hw, cfg, S, p, start)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
