#!/usr/bin/env python3
"""
仲介ノードの割り当て表と換算（SW-65、dev_ws の決定事項58・60）.

**ROS に依存しない純関数。** BR のソフトが使う物理量（ステア角 δ [rad]・車輪の速さ
s [m/s]）と、ros2can の `serial_tx_[ID]`・`serial_rx_[ID]`（Int16 × 24）を行き来する。
値は `config/gateway_map.yaml` から読み、仲介ノード（SW-66）はこの関数群を呼ぶだけにする。

データ構造と流れ:
  module_cmd（δ [rad]・s [m/s]） ──encode_run──▶ {デバイス ID: 24個} ──▶ serial_tx_[ID]
  module_state（δ [rad]・s [m/s]）◀─module_state── decode ◀── serial_rx_[ID]
  止める指令: encode_relax（全部 0＝脱力）/ encode_hold（駆動は 0、ステアは現在角＝フリーズ）
  送信判定:   Gate.step（run / hold / relax）

符号と向き:
  - モータ位置 [rad] = zero + sign * steer.gear * δ
    （zero は原点取得時に決まるモータ位置。電源投入時が 0 のため原点を取ったノードが渡す）
  - 出力軸の速さ [rad/s] = sign * s / wheel_radius * drive.gear

使い方:
  python3 br_ros2can_gateway/scripts/gateway_map.py
  python3 br_ros2can_gateway/scripts/verify_gateway_map.py
"""

from dataclasses import dataclass, field
import math
import os

import yaml

DEFAULT_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'gateway_map.yaml'))

N_SLOTS = 24            # serial_tx_[ID]・serial_rx_[ID] の長さ
PER_MOTOR = 6           # MODE_DAMIAO は1台あたり 6 スロット
MAX_MOTORS = 4          # ESP32 1台に4台まで
MODE_RELAX, MODE_VEL, MODE_POS = 0, 1, 2
ST_RELAX, ST_ON, ST_LATCHED = 0, 1, 15
POS_LSB, VEL_LSB, TORQUE_LSB, INTERNAL_LSB = 0.001, 0.01, 0.001, 0.1
POS_WIDTH = 25.0        # 帰還の位置は ±12.5 rad で折り返す（幅 25 rad＝25000 刻み）
INTERNAL_UNKNOWN = -32768
INT16_MIN, INT16_MAX = -32768, 32767
MIN_RATE_HZ = 50.0      # ros2can の tx_watchdog_sec（0.2 s）に対して 10 周期分
TOP = (
    'rate_hz', 'cmd_timeout', 'fb_timeout', 'wheel_radius', 'drive', 'steer',
    'runaway', 'devices', 'motors', 'inputs', 'input_rules', 'provisional',
)
# 暴走の見張りで「ほぼ 0」とみなす速さ [rad/s、出力軸]（DM3519 の低速時の読みばらつきより大きく取る）
RUNAWAY_ZERO = 1.0
INPUT_KEYS = ('start', 'team', 'estop', 'freeze', 'photo', 'enc')
SWITCH_KEYS = ('start', 'team', 'estop', 'freeze')  # スイッチ入力の口（[ノード, SW番号]）
# 入力用 ESP32（MODE_CAN_HOST）の帰還配列。device_profiles.make_can_host_profile の既定:
# CAN ノード（1〜4、CAN_ID 101〜104）ごとに5スロットで頭から SW1・SW2・SW3・ENC1・ENC2。
# SW は押すと 1（INPUT_PULLUP で !digitalRead）。ノード数・スロット数を変えたらここも直す
CAN_HOST_NODES, CAN_HOST_PER_NODE, CAN_HOST_SW = 4, 5, 3
START_DEBOUNCE = 0.05   # スイッチの読みがこの時間続いたら押下・解放と確定 [s]（仮）
TEAMS = ('red', 'blue')
INPUT_LOST = ('ignore', 'hold', 'relax')  # 入力帰還の途絶時：無視・速度 0 保持・脱力


@dataclass(frozen=True)
class Motor:
    """1輪分のステアまたは駆動モータの配線対応."""

    module: int
    index: int
    sign: int
    home: float = 0.0   # ステアのみ：フォトセンサが切り替わる位置のステア角 [rad]


@dataclass(frozen=True)
class Map:
    """割り当て表（gateway_map.yaml）の内容と単位付きパラメータ一式."""

    rate_hz: float
    cmd_timeout: float
    fb_timeout: float
    wheel_radius: float
    drive_gear: float
    drive_vel_max: float
    steer_gear: float
    steer_vel_limit: float
    steer_pos_limit: object     # None は範囲なし [rad]
    runaway_diff: float
    runaway_over: float
    runaway_hold: float
    runaway_shrink: float
    dev_steer: int
    dev_drive: int
    dev_inputs: int
    steer: tuple                # Motor をモジュール番号の順に並べたタプル
    drive: tuple
    inputs: dict
    # 入力の読み方（SW-71、2026-11-25。input_rules）
    team_on: str = 'blue'       # 陣営スイッチが 1 のときの陣営（0 なら反対）
    estop_on: int = 0           # 非常停止ボタン押下時の読み（0 は NC：押下・断線で 0）
    freeze_on: int = 1          # フリーズ受信機が「止めろ」を出すときの読み
    input_debounce: float = START_DEBOUNCE
    input_lost: str = 'ignore'  # 入力の帰還が fb_timeout 来ないときの扱い（INPUT_LOST）
    provisional: tuple = field(default=())

    @property
    def n(self):
        """モジュール数."""
        return len(self.steer)


def _keys(path, where, d, allowed, required=None):
    unknown = sorted(set(d) - set(allowed))
    if unknown:
        raise ValueError(f'{path}: {where} に知らないキー {unknown}')
    missing = sorted(set(allowed if required is None else required) - set(d))
    if missing:
        raise ValueError(f'{path}: {where} に足りないキー {missing}')


def _motors(path, role, rows, with_home):
    allowed = ('module', 'index', 'sign') + (('home',) if with_home else ())
    out = []
    for r in rows:
        _keys(path, f'motors.{role}', r, allowed)
        if r['sign'] not in (1, -1):
            raise ValueError(f"{path}: motors.{role} の sign は 1 か -1（{r['sign']}）")
        if not 0 <= int(r['index']) < MAX_MOTORS:
            raise ValueError(
                f"{path}: motors.{role} の index は 0〜{MAX_MOTORS - 1}（{r['index']}）")
        out.append(Motor(int(r['module']), int(r['index']), int(r['sign']),
                         float(r.get('home', 0.0))))
    if len(out) > MAX_MOTORS:
        raise ValueError(f'{path}: motors.{role} が {len(out)} 台。ESP32 1台に {MAX_MOTORS} 台まで')
    if len({m.index for m in out}) != len(out):
        raise ValueError(f'{path}: motors.{role} の index が重なっている（スロットが重なる）')
    out.sort(key=lambda m: m.module)
    if [m.module for m in out] != list(range(len(out))):
        raise ValueError(f'{path}: motors.{role} の module は 0 から続き番号で1台ずつ')
    return tuple(out)


def load(path=None, **override):
    """YAML から割り当て表を読む（知らないキー・欠落・異常値は拒否、override は試験用）."""
    path = path or DEFAULT_PATH
    with open(path, encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
    d.update(override)
    _keys(path, '表', d, TOP)
    _keys(path, 'drive', d['drive'], ('gear', 'vel_max'))
    _keys(path, 'steer', d['steer'], ('gear', 'vel_limit', 'pos_limit'))
    _keys(path, 'runaway', d['runaway'], ('reverse_diff', 'over_speed', 'hold', 'shrink'))
    _keys(path, 'devices', d['devices'], ('steer', 'drive', 'inputs'))
    _keys(path, 'motors', d['motors'], ('steer', 'drive'))
    _keys(path, 'inputs', d['inputs'], INPUT_KEYS)
    _keys(path, 'input_rules', d['input_rules'],
          ('team_on', 'estop_on', 'freeze_on', 'debounce', 'lost'))
    rules = d['input_rules']
    if rules['team_on'] not in TEAMS:
        raise ValueError(f"{path}: input_rules.team_on は red か blue（{rules['team_on']}）")
    for name in ('estop_on', 'freeze_on'):
        if type(rules[name]) is not int or rules[name] not in (0, 1):   # True == 1 のため型で見る
            raise ValueError(f'{path}: input_rules.{name} は 0 か 1（{rules[name]}）')
    if rules['lost'] not in INPUT_LOST:
        raise ValueError(
            f"{path}: input_rules.lost は {'・'.join(INPUT_LOST)} のどれか（{rules['lost']}）")
    if not 0.0 < float(rules['debounce']) < 1.0:
        raise ValueError(f"{path}: input_rules.debounce は 0〜1 s（{rules['debounce']}）")
    lim = d['steer']['pos_limit']
    m = Map(
        rate_hz=float(d['rate_hz']), cmd_timeout=float(d['cmd_timeout']),
        fb_timeout=float(d['fb_timeout']), wheel_radius=float(d['wheel_radius']),
        drive_gear=float(d['drive']['gear']), drive_vel_max=float(d['drive']['vel_max']),
        steer_gear=float(d['steer']['gear']), steer_vel_limit=float(d['steer']['vel_limit']),
        steer_pos_limit=None if lim is None else float(lim),
        runaway_diff=float(d['runaway']['reverse_diff']),
        runaway_over=float(d['runaway']['over_speed']),
        runaway_hold=float(d['runaway']['hold']), runaway_shrink=float(d['runaway']['shrink']),
        dev_steer=int(d['devices']['steer']), dev_drive=int(d['devices']['drive']),
        dev_inputs=int(d['devices']['inputs']),
        steer=_motors(path, 'steer', d['motors']['steer'], True),
        drive=_motors(path, 'drive', d['motors']['drive'], False),
        inputs=dict(d['inputs']), team_on=rules['team_on'], estop_on=int(rules['estop_on']),
        freeze_on=int(rules['freeze_on']), input_debounce=float(rules['debounce']),
        input_lost=rules['lost'], provisional=tuple(d.get('provisional') or ()),
    )
    for name in ('rate_hz', 'cmd_timeout', 'fb_timeout', 'wheel_radius', 'drive_gear',
                 'drive_vel_max', 'steer_gear', 'steer_vel_limit', 'runaway_diff',
                 'runaway_over', 'runaway_hold', 'runaway_shrink'):
        if not getattr(m, name) > 0:
            raise ValueError(f'{path}: {name} は正の値にする（{getattr(m, name)}）')
    if m.rate_hz < MIN_RATE_HZ:
        raise ValueError(
            f'{path}: rate_hz {m.rate_hz} は {MIN_RATE_HZ:.0f} Hz 以上にする'
            '（ros2can の途絶の見張りに掛かる）')
    if len({m.dev_steer, m.dev_drive, m.dev_inputs}) != 3:
        raise ValueError(f'{path}: devices の ID が重なっている')
    slots = {}
    for name in SWITCH_KEYS:
        v = m.inputs[name]
        if v is None:
            continue
        if (not isinstance(v, (list, tuple)) or len(v) != 2
                or not all(isinstance(x, int) for x in v)
                or not 1 <= v[0] <= CAN_HOST_NODES or not 1 <= v[1] <= CAN_HOST_SW):
            raise ValueError(
                f'{path}: inputs.{name} は null か '
                f'[ノード 1〜{CAN_HOST_NODES}, SW 1〜{CAN_HOST_SW}]（{v}）')
        slot = switch_slot(m, name)
        if slot in slots:
            raise ValueError(f'{path}: inputs.{name} と inputs.{slots[slot]} が同じ口')
        slots[slot] = name
    if len(m.steer) != len(m.drive) or len(m.steer) < 2:
        raise ValueError(f'{path}: ステアと駆動は同じ台数で、2モジュール以上要る')
    if m.drive_vel_max / VEL_LSB > INT16_MAX or m.steer_vel_limit / VEL_LSB > INT16_MAX:
        raise ValueError(f'{path}: 速さの上限がスロット（0.01 rad/s × 32767）に入らない')
    if m.steer_pos_limit is not None:
        if m.steer_pos_limit / POS_LSB > INT16_MAX:
            raise ValueError(f'{path}: steer.pos_limit がスロット（0.001 rad × 32767）に入らない')
        if m.steer_pos_limit < math.pi / 2.0 * m.steer_gear:
            # ±90° に届かないと、車輪を逆に回しても向けられないステア角が残る
            raise ValueError(
                f'{path}: steer.pos_limit {m.steer_pos_limit} は π/2 × gear '
                f'（{math.pi / 2.0 * m.steer_gear:.2f}）以上にする')
    return m


def switch_slot(m, name):
    """スイッチ name が入力用 ESP32 の帰還の何番目に入るか返す（未定なら None）."""
    v = m.inputs.get(name)
    if v is None:
        return None
    node, sw = v
    return (node - 1) * CAN_HOST_PER_NODE + (sw - 1)


def switch_pressed(m, frame, name):
    """入力用 ESP32 の帰還（24個）からスイッチ name が押されているか返す（読めなければ None）."""
    slot = switch_slot(m, name)
    if slot is None or frame is None or len(frame) != N_SLOTS:
        return None
    return frame[slot] != 0


def switches(m):
    """割り当て表で口が決まっているスイッチ名の列を SWITCH_KEYS 順で返す."""
    return [name for name in SWITCH_KEYS if switch_slot(m, name) is not None]


def estop_held(m, frame):
    """
    配線の非常停止ボタンが押されているか返す（SW-71、未定・読めないときは None）.

    読みの向きは input_rules.estop_on。0（ノーマルクローズ）なら押下と断線の両方が True。
    """
    v = switch_pressed(m, frame, 'estop')
    return None if v is None else int(v) == m.estop_on


def freeze_wanted(m, frame):
    """フリーズ受信機が「止めろ」を出しているか返す（SW-71、【11.4】）."""
    v = switch_pressed(m, frame, 'freeze')
    return None if v is None else int(v) == m.freeze_on


def team_of(m, frame):
    """陣営スイッチの読み（'red' または 'blue'）を返す（読めないときは None、dev_ws の決定事項52）."""
    v = switch_pressed(m, frame, 'team')
    if v is None:
        return None
    return m.team_on if v else next(t for t in TEAMS if t != m.team_on)


class Level:
    """
    入力のばたつきを除いて確定状態を保持するデバウンサ（SW-71）.

    - 同じ読みが debounce [s] 続いたら状態を確定する（確定前は None）
    - 読みが None（帰還途絶）の間は前回状態を維持する（途絶の扱いは input_rules.lost）
    - step は (確定状態, 今回変化したか) を返す
    """

    def __init__(self, debounce=START_DEBOUNCE):
        self.debounce = debounce
        self.state = None
        self.cand = None
        self.cand_t = None

    def step(self, now, value):
        """時刻 now [s] の読み value を1歩進めて (状態, 変化フラグ) を返す."""
        if value is None:
            self.cand = None
            return self.state, False
        if value != self.cand:
            self.cand, self.cand_t = value, now
        if now - self.cand_t < self.debounce or value == self.state:
            return self.state, False
        self.state = value
        return value, True


class StartSwitch:
    """
    スタートスイッチの立ち上がりエッジを1回ずつ取り出すデバウンサ（SW-71）.

    - 離した読みが debounce [s] 続いた後に、押した読みが debounce 続いたら1回だけ True
    - 起動時から押されていた場合は、一度離すまで数えない（短絡・押しっぱなし対策）
    - 読みが None（帰還途絶）の間は押下とも解放とも数えない
    """

    def __init__(self, debounce=START_DEBOUNCE):
        self.debounce = debounce
        self.state = None           # 確定状態（True 押下・False 解放・None 未確定）
        self.cand = None            # 現在継続中の読み
        self.cand_t = None

    def step(self, now, pressed):
        """時刻 now [s] の押下フラグ pressed を進め、押下確定の1拍だけ True を返す."""
        if pressed is None:
            self.cand = None
            return False
        if pressed != self.cand:
            self.cand, self.cand_t = pressed, now
        if now - self.cand_t < self.debounce or pressed == self.state:
            return False
        prev, self.state = self.state, pressed
        return pressed and prev is False


def require_real(m):
    """実機の形で起動してよいか検査し、仮の値が残っていれば ValueError を送出する."""
    if m.provisional:
        raise ValueError(
            f"仮の値が残っているので、実機の形では起動しない：{', '.join(m.provisional)}")


def wrap(a):
    """角度 a [rad] を (−π, π] へ正規化する."""
    return math.atan2(math.sin(a), math.cos(a))


def to_int16(x):
    """値 x を丸めて int16 範囲に収め、(値, 飽和したか) を返す."""
    v = int(round(x))
    if v > INT16_MAX:
        return INT16_MAX, True
    if v < INT16_MIN:
        return INT16_MIN, True
    return v, False


def _blank(m):
    return {m.dev_steer: [0] * N_SLOTS, m.dev_drive: [0] * N_SLOTS}


# ── 指令：物理量 → スロット ────────────────────────────────────────
def wheel_to_motor(m, mot, s):
    """車輪周速 s [m/s] を出力軸角速度 [rad/s] へ変換する."""
    return mot.sign * s / m.wheel_radius * m.drive_gear


def motor_to_wheel(m, mot, w):
    """出力軸角速度 w [rad/s] を車輪周速 [m/s] へ変換する."""
    return mot.sign * w / m.drive_gear * m.wheel_radius


def steer_target(m, mot, delta, pos_now, zero):
    """
    ステア角目標 δ [rad] からモータ位置目標 [rad] と状態印 (tgt, flag) を計算する.

    同じ向きを表す位置は 2π × gear ごとに存在するため現在位置から最も近い値を選ぶ。
    範囲（steer.pos_limit）の外なら範囲内で最も近い位置へ巻き戻す（印 'unwind'）。
    同じ向きが範囲に無ければ半周ずらした位置へ向け、車輪を逆回転させる（印 'flip'）。
    範囲なし（null）でも int16 スロット端（±32.767 rad）で巻き戻す（2026-11-15 対策）。
    """
    turn = 2.0 * math.pi * m.steer_gear
    base = zero + mot.sign * m.steer_gear * wrap(delta)
    k = round((pos_now - base) / turn)
    lim = INT16_MAX * POS_LSB if m.steer_pos_limit is None else m.steer_pos_limit
    cands = [base + (k + j) * turn for j in (-2, -1, 0, 1, 2)]
    inside = [c for c in cands if abs(c) <= lim + 1e-9]
    if inside:
        tgt = min(inside, key=lambda c: abs(c - pos_now))
        return tgt, None if abs(tgt - (base + k * turn)) < 1e-9 else 'unwind'
    # ±135° のように範囲が1周未満のとき、原点ずれで同位相が範囲外なら半周反転する
    flipped = [base + (k + j + 0.5) * turn for j in (-3, -2, -1, 0, 1, 2)]
    inside = [c for c in flipped if abs(c) <= lim + 1e-9]
    if inside:
        return min(inside, key=lambda c: abs(c - pos_now)), 'flip'
    return max(-lim, min(lim, pos_now)), 'range'


def encode_run(m, deltas, speeds, steer_pos, zeros):
    """
    走行指令を {デバイス ID: 24スロット} と印一覧 [(module, flag)] に変換する.

    deltas [rad]・speeds [m/s] はモジュール番号順、steer_pos・zeros は展開済み位置 [rad]。
    """
    out, flags = _blank(m), []
    flips = set()
    for mot, d, pos, zero in zip(m.steer, deltas, steer_pos, zeros):
        tgt, flag = steer_target(m, mot, d, pos, zero)
        if flag:
            flags.append((mot.module, flag))
        if flag == 'flip':
            flips.add(mot.module)
        o = mot.index * PER_MOTOR
        out[m.dev_steer][o:o + 3] = [
            MODE_POS, to_int16(tgt / POS_LSB)[0],
            to_int16(m.steer_vel_limit / VEL_LSB)[0],
        ]
    for mot, s in zip(m.drive, speeds):
        w = wheel_to_motor(m, mot, -s if mot.module in flips else s)
        if abs(w) > m.drive_vel_max:
            w = math.copysign(m.drive_vel_max, w)
            flags.append((mot.module, 'clip'))
        o = mot.index * PER_MOTOR
        out[m.dev_drive][o:o + 2] = [MODE_VEL, to_int16(w / VEL_LSB)[0]]
    return out, flags


def encode_hold(m, steer_pos):
    """フリーズ指令を作る：駆動は速度 0、ステアは現在位置を保持する（dev_ws の決定事項60）."""
    out = _blank(m)
    lim = m.steer_pos_limit
    for mot, pos in zip(m.steer, steer_pos):
        if lim is not None:
            pos = max(-lim, min(lim, pos))
        o = mot.index * PER_MOTOR
        out[m.dev_steer][o:o + 3] = [
            MODE_POS, to_int16(pos / POS_LSB)[0],
            to_int16(m.steer_vel_limit / VEL_LSB)[0],
        ]
    for mot in m.drive:
        o = mot.index * PER_MOTOR
        out[m.dev_drive][o:o + 2] = [MODE_VEL, 0]
    return out


def encode_relax(m):
    """脱力指令（全スロット 0）を作る。停止条件成立時と終了時に送り ESP32 ラッチも解除する."""
    return _blank(m)


def drive_targets(m, frames):
    """送信フレームから駆動の速度目標 [rad/s, 出力軸] をモジュール番号順で取り出す."""
    fr = frames[m.dev_drive]
    return [fr[mot.index * PER_MOTOR + 1] * VEL_LSB for mot in m.drive]


# ── 帰還：スロット → 物理量 ────────────────────────────────────────
def decode(m, frames):
    """
    帰還フレーム辞書から {'steer': [...], 'drive': [...]} を復号する（単位 SI）.

    1台分は status・count・internal [rad]・pos [rad] (±12.5 折り返し)・vel [rad/s]・torque [N·m]。
    """
    out = {}
    for role, dev, mots in (('steer', m.dev_steer, m.steer), ('drive', m.dev_drive, m.drive)):
        fr = frames.get(dev)
        rows = []
        for mot in mots:
            if fr is None:
                rows.append(None)
                continue
            if len(fr) != N_SLOTS:
                raise ValueError(f'serial_rx_{dev} の長さが {len(fr)}（{N_SLOTS} のはず）')
            s = fr[mot.index * PER_MOTOR:(mot.index + 1) * PER_MOTOR]
            rows.append({
                'status': int(s[0]), 'count': int(s[1]),
                'internal': None if s[2] == INTERNAL_UNKNOWN else s[2] * INTERNAL_LSB,
                'pos': s[3] * POS_LSB, 'vel': s[4] * VEL_LSB, 'torque': s[5] * TORQUE_LSB,
            })
        out[role] = rows
    return out


def unwrap(prev_total, pos):
    """
    折り返し位置 pos [rad] を前回の連続値 prev_total [rad] に続けて展開する（幅 25 rad）.

    ros2can の `serial_rx_[ID]_unwrapped` は 32768 刻みで折り返すため 7.768 rad ずれるので使わない。
    """
    return prev_total + (pos - prev_total + POS_WIDTH / 2.0) % POS_WIDTH - POS_WIDTH / 2.0


def seed_total(pos, internal):
    """起動初回の展開位置 [rad] を internal（0.1 rad 刻み多回転位置）から決める."""
    if internal is None:
        return pos
    return pos + round((internal - pos) / POS_WIDTH) * POS_WIDTH


def module_state(m, dec, steer_totals, zeros):
    """
    帰還から /drive/module_state 用の (δ の列 [rad], s の列 [m/s]) を組み立てる.

    DM3519 の速度帰還は低速で約 25 % 低めに出る実測があるため、s はオドメトリには使わない。
    """
    deltas = [
        wrap(mot.sign * (tot - zero) / m.steer_gear)
        for mot, tot, zero in zip(m.steer, steer_totals, zeros)
    ]
    speeds = [
        motor_to_wheel(m, mot, row['vel']) if row else 0.0
        for mot, row in zip(m.drive, dec['drive'])
    ]
    return deltas, speeds


def zero_from_home(m, mot, pos_at_home):
    """フォトセンサ検出位置 pos_at_home [rad] から δ=0 時のモータ位置 zero [rad] を求める."""
    return pos_at_home - mot.sign * m.steer_gear * mot.home


# ── 駆動の暴走の見張り ──────────────────────────────────────────
class RunawayWatch:
    """
    DM3519 の暴走を監視する純状態機械（2026-11-18 BLDC 担当仕様）.

    11/9 に指令と逆向きに全速回転し続ける暴走が故障コード無しで2回発生したための安全対策。
    各輪について逆向き／指令超過（差が reverse_diff 以上）または上限超過（over_speed 以上）が
    hold [s] 続いたら暴走と判定する。通常の加減速で差が shrink 以上縮む間は計測し直す。
    """

    def __init__(self, m):
        self.m = m
        self.reset()

    def reset(self):
        """監視タイマと前回誤差を初期化する."""
        self.since = [None] * self.m.n
        self.err0 = [0.0] * self.m.n

    def step(self, now, targets, actuals):
        """1周期分の監視を行い、暴走と判定したモジュール番号（無ければ None）を返す."""
        m, hit = self.m, None
        for i, (cmd, vel) in enumerate(zip(targets, actuals)):
            err = abs(cmd - vel)
            over = abs(vel) >= m.drive_vel_max + m.runaway_over
            wrong = (
                err >= m.runaway_diff and abs(vel) >= RUNAWAY_ZERO
                and (cmd * vel < 0.0 or abs(cmd) < RUNAWAY_ZERO or abs(vel) > abs(cmd))
            )
            if not (over or wrong):
                self.since[i] = None
            elif self.since[i] is None or (not over and err <= self.err0[i] - m.runaway_shrink):
                self.since[i], self.err0[i] = now, err
            elif now - self.since[i] >= m.runaway_hold and hit is None:
                hit = i
        return hit


# ── どれを送るか ──────────────────────────────────────────────
class Gate:
    """
    指令モード（run / hold / relax）を決める純状態機械.

    - 起動直後は relax。arm で異常が無ければ run に遷移
    - 非常停止・途絶・故障・異常で relax へ落とし、原因解消後も再 arm まで維持
    - フリーズ時は hold（速度 0・現在角度保持）し、解除後は再 arm なしで run に復帰
    """

    def __init__(self, m):
        self.m = m
        self.armed = False
        self.reason = 'startup'

    def _trip(self, estop, cmd_age, fb_age, statuses):
        if estop:
            return 'estop'
        if fb_age is None or fb_age > self.m.fb_timeout:
            return 'fb_lost'
        bad = [s for s in statuses if s not in (ST_RELAX, ST_ON)]
        if bad:
            return 'latched' if all(s == ST_LATCHED for s in bad) else f'fault:{bad[0]}'
        if cmd_age is None or cmd_age > self.m.cmd_timeout:
            return 'cmd_lost'
        return None

    def step(self, *, arm=False, estop=False, freeze=False,
             cmd_age=None, fb_age=None, statuses=(), extra=None):
        """1周期分の (モード, 理由) を返す（cmd_age・fb_age [s]、extra はノード側の停止要因）."""
        trip = self._trip(estop, cmd_age, fb_age, statuses)
        if extra and trip != 'estop':
            trip = extra        # 非常停止の次に強い（ノード側で検出した理由を残す）
        if trip:
            self.armed, self.reason = False, trip
            return 'relax', trip
        if not self.armed:
            if not arm:
                return 'relax', self.reason
            self.armed, self.reason = True, None
        if freeze:
            return 'hold', 'freeze'
        return 'run', None


def describe(m):
    """割り当て表 m の構成・制限値・仮置き項目を人間向けの文字列に整形する."""
    lim = ('範囲なし' if m.steer_pos_limit is None else
           f'±{m.steer_pos_limit:g} rad'
           f'（ステア軸 ±{math.degrees(m.steer_pos_limit / m.steer_gear):.0f}°）')
    top = m.drive_vel_max / m.drive_gear * m.wheel_radius
    lines = [
        f'周期 {m.rate_hz:g} Hz・上流の途絶 {m.cmd_timeout:g} s・帰還の途絶 {m.fb_timeout:g} s',
        f'駆動：serial_tx_{m.dev_drive}、車輪の最高周速 {top:.3f} m/s'
        f'（出力軸 {m.drive_vel_max:g} rad/s）',
        f'ステア：serial_tx_{m.dev_steer}、位置の範囲 {lim}、速さの上限 {m.steer_vel_limit:g} rad/s',
    ]
    for role, mots in (('ステア', m.steer), ('駆動', m.drive)):
        lines.append(f'{role}：' + '、'.join(
            f'モジュール {x.module} → '
            f'スロット {x.index * PER_MOTOR}〜{x.index * PER_MOTOR + PER_MOTOR - 1}'
            f'（向き {x.sign:+d}）' for x in mots))
    lines.append(f'入力：serial_rx_{m.dev_inputs}、' + '、'.join(
        f'{name} ' + ('未定' if switch_slot(m, name) is None
                      else f'N{m.inputs[name][0]} SW{m.inputs[name][1]}'
                           f' → スロット {switch_slot(m, name)}')
        for name in SWITCH_KEYS))
    lines.append(
        f'入力の読み方：陣営スイッチが 1 で {m.team_on}・非常停止のボタンは {m.estop_on} '
        f'で押している・フリーズの受信機は {m.freeze_on} で止めろ・ばたつき '
        f'{m.input_debounce:g} s・途切れたら {m.input_lost}')
    lines.append('仮の値：' + ('、'.join(m.provisional) if m.provisional else '無し'))
    return '\n'.join(lines)


if __name__ == '__main__':
    print(describe(load()))
