#!/usr/bin/env python3
"""
仲介ノード（SW-66、dev_ws の決定事項58・60・61・67）.

BR の制御ソフトと ros2can の間に1つだけ置き、換算・暴走監視・安全停止・入力読みを担う。
純計算・状態機械は `gateway_map.py` に分離し、このノードは時計と ROS 入出力だけを持つ。

入出力:
  sub  /drive/module_cmd     (std_msgs/Float64MultiArray, [δ0..δ3[rad], s0..s3[m/s]])
  sub  /serial_rx_<dev>      (std_msgs/Int16MultiArray, 24要素: ステア/駆動/入力 ESP32 帰還)
  sub  /mission_fsm/estop    (std_msgs/Empty, 非常停止トリガ)
  sub  /mission_fsm/freeze   (std_msgs/Bool, フリーズ要求)
  pub  /serial_tx_<dev>      (std_msgs/Int16MultiArray, 24要素: 単一送信元を強制)
  pub  /drive/module_state   (std_msgs/Float64MultiArray, [δ0..δ3[rad], s0..s3[m/s]])
  pub  /hw/status            (std_msgs/String, JSON 状態)
  pub  /hw/team              (std_msgs/String, 'red'/'blue'/'' ラッチ配信)
  pub  /mission_fsm/estop    (std_msgs/Empty, 配線の非常停止ボタン押下中 0.5 s 周期再送)
  pub  /mission_fsm/freeze   (std_msgs/Bool, フリーズ受信機変化時に層へ伝達)
  cli  /mission_fsm/start    (std_srvs/Trigger, スタートスイッチ押下時に呼び出し)
  srv  ~/arm                 (std_srvs/Trigger, 走行許可を与える)

安全仕様:
  - `serial_tx_<dev>` に他ノードの publisher が見つかれば起動拒否／即脱力（other_tx）
  - 上流・帰還の途絶、非常停止、故障、暴走（0.3 s 継続）で脱力し、次回の ~/arm まで維持
  - SIGINT/SIGTERM 時は rclpy 終了前に脱力フレームを5回送出してから閉じる
  - `--hw real` は gateway_map.yaml に仮置き項目が1つでも残っていれば起動しない

使い方:
  ros2 run br_ros2can_gateway gateway_node.py --auto-arm
  ros2 run br_ros2can_gateway gateway_node.py --start-switch
"""

import argparse
import json
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Empty, Float64MultiArray, Int16MultiArray, String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import gateway_map as G                                            # noqa: E402, I100

STATUS_EVERY = 0.2      # 変化が無くても /hw/status を出す間隔 [s]
OTHERS_EVERY = 0.5      # serial_tx_<id> の他送信元を監視する間隔 [s]
ESTOP_EVERY = 0.5       # 配線非常停止ボタン押下中、層へ再送する間隔 [s]
FREEZE_EVERY = 1.0      # フリーズ受信機が止めろの間、層へ再送する間隔 [s]


def tx_topic(dev):
    """デバイス ID に対応する送信トピック名を返す."""
    return f'/serial_tx_{dev}'


def rx_topic(dev):
    """デバイス ID に対応する帰還トピック名を返す."""
    return f'/serial_rx_{dev}'


class Gateway(Node):
    """物理指令 ↔ ros2can スロット変換と安全・入力監視を行う仲介ノード."""

    def __init__(self, args, m):
        super().__init__('br_ros2can_gateway')
        self.args, self.m = args, m
        self.gate = G.Gate(m)
        self.watch = G.RunawayWatch(m)
        self.drive_sent = None          # 最後の駆動速度目標 [rad/s, 出力軸]。脱力時は None
        self.devs = (m.dev_steer, m.dev_drive)
        self.cmd = None                 # 最後の /drive/module_cmd（[δ.., s..]）
        self.cmd_t = None
        self.rx = {}                    # デバイス ID -> 最新の帰還（24要素）
        self.rx_t = {}
        self.totals = [None] * m.n      # 多回転展開済みのステアモータ位置 [rad]
        self.zeros = list(args.zeros)   # δ = 0 時のモータ位置 [rad]
        self.freeze = False
        self.estop_t = None
        self.arm_req = False
        self.auto_left = 1 if args.auto_arm else 0
        self.others = 0                 # serial_tx_[ID] の他送信元の数
        self.others_t = 0.0
        self.sent = None                # 最後に送った状態（run / hold / relax）
        self.ready = False              # 他送信元が無いことを確認するまで送信しない
        self.flag_now = frozenset()
        self.status_t = 0.0
        self.status_last = None

        # ── パブリッシャ・サブスクライバ・サービス ──
        self.tx = {d: self.create_publisher(Int16MultiArray, tx_topic(d), 10) for d in self.devs}
        self.state_pub = self.create_publisher(Float64MultiArray, args.state, 10)
        self.status_pub = self.create_publisher(String, args.status, 10)
        self.create_subscription(Float64MultiArray, args.cmd, self.on_cmd, 10)
        for d in self.devs:
            self.create_subscription(
                Int16MultiArray, rx_topic(d), lambda msg, d=d: self.on_rx(d, msg), 10)
        self.create_subscription(Empty, args.estop, self.on_estop, 10)
        self.create_subscription(Bool, args.freeze, self.on_freeze, 10)
        self.create_service(Trigger, '~/arm', self.on_arm)

        # ── 機体入力（SW-71） ──
        self.switch = G.StartSwitch() if args.start_switch else None
        self.in_rx = None               # 入力用 ESP32 の最新帰還（24要素）
        self.in_rx_t = None
        self.start_pressed = None
        self.start_count = 0
        self.levels = {
            name: G.Level(m.input_debounce) for name in G.switches(m) if name != 'start'
        }
        self.reading = {}               # デバウンス後の確定状態
        self.estop_pub_t = 0.0
        self.freeze_pub_t = 0.0
        self.freeze_rx = False
        self.team_sent = None
        self.lost_logged = False
        if G.switches(m):
            self.create_subscription(Int16MultiArray, rx_topic(m.dev_inputs), self.on_inputs, 10)
        if self.switch is not None:
            self.start_cli = self.create_client(Trigger, args.start_srv)
        if 'estop' in self.levels:
            self.estop_out = self.create_publisher(Empty, args.estop, 10)
        if 'freeze' in self.levels:
            self.freeze_out = self.create_publisher(Bool, args.freeze, 10)
        if 'team' in self.levels:
            latched = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.team_out = self.create_publisher(String, args.team_topic, latched)

        self.create_timer(
            1.0 / m.rate_hz, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if args.auto_arm:
            arm_desc = (
                '上流の指令が最初に 0 でなくなったとき、1回だけ自分で動かす'
                '（--auto-arm、シム専用）')
        elif self.switch is not None:
            slot = G.switch_slot(m, 'start')
            arm_desc = (
                f'スタートスイッチ（{rx_topic(m.dev_inputs)} のスロット {slot}）'
                f'を押すと動かし、{args.start_srv} を呼ぶ')
        else:
            arm_desc = '合図（~/arm）を受けるまで脱力'
        self.get_logger().info(
            f'仲介ノード（{args.hw}）：{args.cmd} -> {tx_topic(m.dev_steer)}・'
            f'{tx_topic(m.dev_drive)}、{m.rate_hz:g} Hz。上流の途絶 {m.cmd_timeout:g} s・'
            f'帰還の途絶 {m.fb_timeout:g} s で脱力。{arm_desc}')

    # ── 受信コールバック ──────────────────────────────────────────────
    def on_cmd(self, msg):
        """逆運動学からの [δ.., s..] 指令を受信する（不正形状・NaN は無視）."""
        if len(msg.data) != 2 * self.m.n or not all(math.isfinite(x) for x in msg.data):
            return
        self.cmd, self.cmd_t = list(msg.data), time.monotonic()

    def on_rx(self, dev, msg):
        """ESP32 帰還を受信し、ステアは毎通多回転位置を展開する."""
        if len(msg.data) != G.N_SLOTS:
            return
        self.rx[dev], self.rx_t[dev] = list(msg.data), time.monotonic()
        if dev != self.m.dev_steer:
            return
        rows = G.decode(self.m, {dev: self.rx[dev]})['steer']
        for i, row in enumerate(rows):
            self.totals[i] = (
                G.seed_total(row['pos'], row['internal']) if self.totals[i] is None
                else G.unwrap(self.totals[i], row['pos'])
            )

    def on_estop(self, _msg):
        """非常停止時刻を更新する."""
        self.estop_t = time.monotonic()

    def on_freeze(self, msg):
        """フリーズ状態を更新する."""
        self.freeze = bool(msg.data)

    def estop_now(self, now):
        """非常停止保持時間内かどうかを返す."""
        return self.estop_t is not None and now - self.estop_t < self.args.estop_hold

    def on_inputs(self, msg):
        """入力用 ESP32 の 24 スロットを受信する."""
        if len(msg.data) == G.N_SLOTS:
            self.in_rx, self.in_rx_t = list(msg.data), time.monotonic()

    def estop_button(self):
        """配線の非常停止ボタンが押下中と確定しているか返す（SW-71）."""
        return self.reading.get('estop') is True

    def inputs_fresh(self, now):
        """入力用 ESP32 の帰還がタイムアウト内か返す."""
        return self.in_rx_t is not None and now - self.in_rx_t < self.m.fb_timeout

    # ── 機体入力の処理（SW-71） ──────────────────────────────────────
    def read_inputs(self, now):
        """陣営・非常停止ボタン・フリーズ受信機を読み取り層へ配信する."""
        m = self.m
        frame = self.in_rx if self.inputs_fresh(now) else None
        reads = {
            'estop': G.estop_held(m, frame),
            'freeze': G.freeze_wanted(m, frame),
            'team': G.team_of(m, frame),
        }
        for name, lv in self.levels.items():
            before = self.reading.get(name)
            state, changed = lv.step(now, reads[name])
            self.reading[name] = state
            if not changed:
                continue
            if name == 'estop':
                if state:
                    self.get_logger().warn(
                        '非常停止のボタン：押している。脱力し、層へ非常停止を出す')
                    self.estop_pub_t = 0.0
                elif before:
                    self.get_logger().info(
                        '非常停止のボタン：離した（スタートスイッチを押すまで動かない）')
            elif name == 'freeze':
                if state:
                    self.get_logger().warn(
                        'フリーズの受信機：止めろ。速度 0 で保ち、層へフリーズを出す')
                    self.freeze_pub_t = 0.0
                    self.freeze_rx = True
                elif self.freeze_rx:
                    self.get_logger().info('フリーズの受信機：再開。層へ再開を出す')
                    self.freeze_rx = False
                    self.freeze = False
                    self.freeze_out.publish(Bool(data=False))

        if self.estop_button() and now - self.estop_pub_t >= ESTOP_EVERY:
            self.estop_pub_t = now
            self.estop_t = time.monotonic()
            self.estop_out.publish(Empty())
        if self.freeze_rx and now - self.freeze_pub_t >= FREEZE_EVERY:
            self.freeze_pub_t = now
            self.freeze = True
            self.freeze_out.publish(Bool(data=True))
        if 'team' in self.levels:
            team = self.reading.get('team') if frame is not None else None
            if team != self.team_sent:
                self.team_sent = team
                self.team_out.publish(String(data=team or ''))
                self.get_logger().info(
                    f"陣営スイッチ：{team or '読めない'}（{self.args.team_topic}）")

    def input_lost(self, now):
        """入力スイッチ口が1つ以上定義されており、帰還が途切れているか返す."""
        return bool(G.switches(self.m)) and not self.inputs_fresh(now)

    def arm_refusal(self, now):
        """合図を拒否する理由を返す（許可可なら None）."""
        if self.estop_button():
            return '非常停止のボタンが押されているので、合図を受けない'
        if self.estop_now(now):
            return '非常停止を受けているので、合図を受けない'
        if self.spinning():
            return '車輪が回っているので、合図を受けない'
        return None

    def on_arm(self, _req, res):
        """~/arm サービス：安全条件を満たしていれば走行許可を与える."""
        why = self.arm_refusal(time.monotonic())
        if why:
            res.success, res.message = False, why
            return res
        self.arm_req = True
        res.success = True
        res.message = (
            '合図を受けた（止める理由が残っていれば動かない。/hw/status を見る）')
        return res

    def read_start_switch(self, now):
        """スタートスイッチ押下を検出し、ゲート許可と層の ~/start 呼び出しを行う."""
        fresh = self.inputs_fresh(now)
        self.start_pressed = G.switch_pressed(self.m, self.in_rx, 'start') if fresh else None
        if not self.switch.step(now, self.start_pressed):
            return
        self.start_count += 1
        why = self.arm_refusal(now)
        if why:
            self.get_logger().warn(
                f'スタートスイッチ：{why}。層の {self.args.start_srv} も呼ばない')
            return
        self.arm_req = True
        if not self.start_cli.service_is_ready():
            self.get_logger().warn(
                f'スタートスイッチ：合図を受けた。層の {self.args.start_srv} が見つからない')
            return
        self.start_cli.call_async(Trigger.Request()).add_done_callback(self.on_start_done)
        self.get_logger().info(
            f'スタートスイッチ：合図を受け、層の {self.args.start_srv} を呼んだ')

    def on_start_done(self, fut):
        """~/start 呼び出し結果をログに記録する."""
        try:
            res = fut.result()
        except Exception as e:                                     # noqa: BLE001
            self.get_logger().error(
                f'スタートスイッチ：層の {self.args.start_srv} が失敗した（{e}）')
            return
        if res.success:
            self.get_logger().info(f'スタートスイッチ：層が受けた（{res.message}）')
        else:
            self.get_logger().warn(f'スタートスイッチ：層が断った（{res.message}）')

    def spinning(self):
        """脱力中に駆動輪が暴走しきい値以上で回転しているか返す."""
        rows = G.decode(self.m, self.rx)['drive']
        return any(r and abs(r['vel']) >= self.m.runaway_diff for r in rows)

    # ── 定周期送信・監視 ──────────────────────────────────────────────
    def publish_frames(self, frames):
        """ステア・駆動の両デバイスへ Int16MultiArray を送信する."""
        for d in self.devs:
            out = Int16MultiArray()
            out.data = [int(v) for v in frames[d]]
            self.tx[d].publish(out)

    def tick(self):
        """状態判定・安全監視・フレーム送信・状態公開を1周期分実行する."""
        if not self.ready:
            return
        now = time.monotonic()
        m = self.m
        if now - self.others_t > OTHERS_EVERY:
            self.others_t = now
            self.others = sum(
                max(self.count_publishers(tx_topic(d)) - 1, 0) for d in self.devs)

        cmd_age = None if self.cmd_t is None else now - self.cmd_t
        fb_age = (
            None if any(d not in self.rx_t for d in self.devs)
            else now - min(self.rx_t[d] for d in self.devs)
        )
        if self.levels:
            self.read_inputs(now)
        if self.switch is not None:
            self.read_start_switch(now)

        lost = self.input_lost(now)
        if lost != self.lost_logged:
            self.lost_logged = lost
            if lost:
                self.get_logger().warn(
                    f'入力の帰還（{rx_topic(m.dev_inputs)}）が来ない（扱い：{m.input_lost}）')
            else:
                self.get_logger().info('入力の帰還が戻った')

        dec = G.decode(m, self.rx)
        statuses = [r['status'] for role in ('steer', 'drive') for r in dec[role] if r]
        # シム用 --auto-arm は初回非ゼロ速度指令の立ち上がりのみ使用
        moving = self.cmd is not None and any(abs(v) > 1e-6 for v in self.cmd[m.n:])
        arm = (self.arm_req or (self.auto_left > 0 and moving)) and not self.spinning()
        self.arm_req = False
        was = self.gate.armed

        extra = 'other_tx' if self.others else None
        if lost and m.input_lost == 'relax' and extra is None:
            extra = 'input_lost'
        if self.drive_sent is not None and all(dec['drive']):
            hit = self.watch.step(now, self.drive_sent, [r['vel'] for r in dec['drive']])
            if hit is not None:
                extra = f'runaway:{hit}'

        freeze = self.freeze or self.freeze_rx or (lost and m.input_lost == 'hold')
        mode, reason = self.gate.step(
            arm=arm, estop=self.estop_now(now) or self.estop_button(),
            freeze=freeze, cmd_age=cmd_age, fb_age=fb_age, statuses=statuses, extra=extra,
        )
        if mode == 'hold' and not (self.freeze or self.freeze_rx):
            reason = 'input_lost'       # 保持理由を入力途絶（input_rules.lost: hold）として明示
        if self.gate.armed and not was:
            self.auto_left = 0

        flags = []
        if mode == 'run':
            frames, flags = G.encode_run(
                m, self.cmd[:m.n], self.cmd[m.n:], self.totals, self.zeros)
        elif mode == 'hold':
            frames = G.encode_hold(m, self.totals)
        else:
            frames = G.encode_relax(m)

        if mode == 'relax':
            self.drive_sent = None
            self.watch.reset()
        else:
            self.drive_sent = G.drive_targets(m, frames)
        self.publish_frames(frames)

        if all(t is not None for t in self.totals) and all(dec['drive']):
            deltas, speeds = G.module_state(m, dec, self.totals, self.zeros)
            st = Float64MultiArray()
            st.data = [float(v) for v in deltas + speeds]
            self.state_pub.publish(st)

        if mode != self.sent:
            # rclpy ロガーは同一呼び出し行からのログレベル混在で例外を出すため分岐を分ける
            if mode == 'run':
                self.get_logger().info('走る')
            elif mode == 'hold':
                self.get_logger().warn(f'保つ（{reason}）')
            else:
                self.get_logger().warn(f'脱力（{reason}）')
            self.sent = mode

        now_flags = frozenset(f'{mod}:{flag}' for mod, flag in flags)
        if now_flags != self.flag_now:
            if now_flags:
                self.get_logger().warn(f"印 {' '.join(sorted(now_flags))}（モジュール:印）")
            else:
                self.get_logger().info('印が消えた')
            self.flag_now = now_flags

        status = {
            'mode': mode, 'reason': reason, 'armed': self.gate.armed, 'pid': os.getpid(),
            'flags': sorted({f'{mod}:{flag}' for mod, flag in flags}),
            'motor_status': statuses, 'others': self.others,
        }
        if self.switch is not None:
            status['start'] = {'pressed': self.start_pressed, 'count': self.start_count}
        if G.switches(m):
            status['inputs'] = {
                **{k: self.reading.get(k) for k in self.levels}, 'lost': lost,
            }
        if status != self.status_last or now - self.status_t > STATUS_EVERY:
            self.status_last, self.status_t = status, now
            out = String()
            payload = {
                **status,
                'cmd_age': None if cmd_age is None else round(cmd_age, 3),
                'fb_age': None if fb_age is None else round(fb_age, 3),
            }
            out.data = json.dumps(payload, ensure_ascii=False)
            self.status_pub.publish(out)

    def other_senders(self, wait):
        """起動時に serial_tx_<dev> の他送信元を wait [s] だけ探索する."""
        end = time.monotonic() + wait
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return {
            tx_topic(d): self.count_publishers(tx_topic(d)) - 1
            for d in self.devs if self.count_publishers(tx_topic(d)) > 1
        }


def floats(text):
    """カンマ区切り文字列を float リストに変換する."""
    return [float(x) for x in text.split(',')]


def main():
    """引数を検証し、他送信元確認のうえ仲介ノードをスピンする."""
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        '--hw', choices=('fake', 'real'), default='fake',
        help='fake は ros2can の模擬（シム）。'
             'real は実機で、割り当て表に仮の値が残っていると起動しない')
    ap.add_argument('--map', default=None, help='割り当て表（既定は config/gateway_map.yaml）')
    ap.add_argument(
        '--auto-arm', action='store_true',
        help='シム専用：上流の指令が最初に 0 でなくなったときに1回だけ、'
             '合図なしで動かす（スタートスイッチの代わり）')
    ap.add_argument(
        '--start-switch', action='store_true',
        help='入力用の ESP32 のスタートスイッチを読み、押したら動かして層の ~/start を呼ぶ'
             '（SW-71）。実機の形ではいつも読む。--auto-arm とは一緒に使えない')
    ap.add_argument(
        '--start-srv', default='/mission_fsm/start',
        help='スタートスイッチで呼ぶ層のサービス（Trigger）')
    ap.add_argument(
        '--zeros', type=floats, default=None,
        help='δ = 0 のステアのモータの位置 [rad]（モジュール番号の順、カンマ区切り）。'
             '既定は全部 0＝電源を入れたときタイヤが前を向いている（模擬の既定）。'
             '実機は原点を取った所が渡す。負の値は --zeros=-1.9,… の形で渡す')
    ap.add_argument(
        '--estop-hold', type=float, default=1.0,
        help='非常停止を1通受けてから、押されているとみなす時間 [s]')
    ap.add_argument(
        '--discover', type=float, default=2.0,
        help='起動のとき、ほかの送り手を探す時間 [s]。'
             'この間は何も送らない（発見に 1 s を超えることがある）')
    ap.add_argument('--cmd', default='/drive/module_cmd')
    ap.add_argument('--state', default='/drive/module_state')
    ap.add_argument('--status', default='/hw/status')
    ap.add_argument('--estop', default='/mission_fsm/estop')
    ap.add_argument('--freeze', default='/mission_fsm/freeze')
    ap.add_argument(
        '--team-topic', default='/hw/team',
        help='陣営スイッチの読みを出すトピック（std_msgs/String）')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    try:
        m = G.load(args.map)
        if args.hw == 'real':
            G.require_real(m)
            if args.auto_arm:
                raise ValueError('--auto-arm はシム専用（実機ではスタートスイッチで動かす）')
            if args.zeros is None:
                raise ValueError(
                    '実機では原点（--zeros）が要る。'
                    '電源を入れた所が 0 なので、決め打ちにしない')
            args.start_switch = True    # 実機の合図はスタートスイッチ（dev_ws の決定事項61）
        if args.start_switch and args.auto_arm:
            raise ValueError(
                '--start-switch と --auto-arm は一緒に使えない（合図の出し手は1つ）')
        if args.start_switch and G.switch_slot(m, 'start') is None:
            raise ValueError(
                '割り当て表の inputs.start が未定（null）なので、スタートスイッチを読めない')
        if args.zeros is None:
            args.zeros = [0.0] * m.n
        if len(args.zeros) != m.n:
            raise ValueError(f'--zeros は {m.n} 個')
    except ValueError as e:
        print(f'仲介ノード：起動しない。{e}', file=sys.stderr)
        return 2

    # 終了時に脱力フレームを送り切るため、SIGINT・SIGTERM を自前で受ける
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    node = Gateway(args, m)
    code = 0
    try:
        others = node.other_senders(args.discover)
        if others:
            node.get_logger().error(
                f'起動しない：ほかの送り手がいる {others}。'
                'ros2can のスロットへ送るのは仲介ノードだけ')
            code = 3
        node.ready = not code
        while not stop and not code and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if code == 0 and rclpy.ok():
            for _ in range(5):
                node.publish_frames(G.encode_relax(m))
                time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
