#!/usr/bin/env python3
"""
4輪独立ステアの状態方程式・運動学・制約（SW-44、ROS 非依存純関数）.

    ros2 run br_ros2can_gateway swerve.py       # 読んだ値と上限目安を表示
    python3 br_ros2can_gateway/scripts/swerve.py --params 別の.yaml

軌道生成（SW-45）・追従器（SW-48）・シムモジュール模型（SW-47）と仲介ノード（SW-65）が
同一関数と `config/swerve.yaml` を共有する（本モジュール内では casadi を import しない）。

座標・単位・符号規約:
  - 単位は [m], [rad], [s]。状態は `map`、機体速度とモジュール位置は `base_link` (x 前, y 左)
  - モジュール順は `0: 左前, 1: 左後, 2: 右後, 3: 右前`
  - ステア角 delta [rad] は x 軸基準反時計回り（delta + pi と -s は同値反転）
"""

import argparse
from dataclasses import dataclass, fields
import math
import os
from types import SimpleNamespace

import numpy as np
import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_PATH = os.path.normpath(os.path.join(HERE, '..', 'config', 'swerve.yaml'))
N_POINT = 6        # 状態の「点」の部分 [x, y, θ, vx, vy, ω]
N_ACCEL = 3        # 入力の「点」の部分 [ax, ay, α]
# 範囲つきのステアで、対角の2輪を先に振り直す決まり（`ik_targets`。2026-11-19）。端までの余裕のしきい値は
# swerve.yaml の steer_early_room・steer_early_room_drift
OM_DRIFT = 0.2                       # これより遅い回転は、流れる向きが分からないとみなす [rad/s]


def backend(mod):
    """Tri-func backend 差分（numpy の arctan2/abs vs CasADi の atan2/fabs）を吸収する."""
    def pick(*names):
        return next(getattr(mod, n) for n in names if hasattr(mod, n))
    return SimpleNamespace(cos=mod.cos, sin=mod.sin, sqrt=mod.sqrt,
                           atan2=pick('atan2', 'arctan2'), fabs=pick('fabs', 'abs'))


NP = backend(np)


@dataclass(frozen=True)
class Params:
    """足回りパラメータ群（`load` で YAML から構築する仮値）."""

    modules: tuple                  # ((x, y), …) [m]。base_link
    wheel_speed_max: float          # 車輪の最高周速 [m/s]
    steer_rate_max: float           # ステアの最高角速度 [rad/s]
    steer_range: object             # ステア角の範囲 [rad]。None は制限なし
    speed_max: float                # 速度ノルム [m/s]
    accel_max: float                # 加速度ノルム [m/s^2]
    omega_max: float                # 角速度 [rad/s]
    alpha_max: float                # 角加速度 [rad/s^2]
    wheel_stop: float = 1e-3        # これより遅い車輪はステア角を保つ [m/s]
    steer_early_speed: float = 0.3  # これより速いと対角2輪を先に振り直す [m/s]
    # 流れる向き不明時／判明時の事前振替しきい値 [rad]
    steer_early_room: float = math.radians(30)
    steer_early_room_drift: float = math.radians(75)

    @property
    def n(self):
        return len(self.modules)


# YAML に無くてもよいキー（Params の既定値を使う）
OPTIONAL = ('wheel_stop', 'steer_early_speed', 'steer_early_room', 'steer_early_room_drift')


def load(path=None, **override):
    """
    YAML からパラメータを読み込み未知キー・範囲外値を拒否して `Params` を返す.

    `override` は試験用の値上書きに使う。
    """
    path = path or DEFAULT_PATH
    with open(path, encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
    d.update(override)
    names = {f.name for f in fields(Params)}
    unknown = sorted(set(d) - names)
    if unknown:
        raise ValueError(f'{path}: 知らないキー {unknown}')
    missing = sorted(f.name for f in fields(Params)
                     if f.name not in d and f.name not in OPTIONAL)
    if missing:
        raise ValueError(f'{path}: 足りないキー {missing}')
    mods = tuple((float(x), float(y)) for x, y in d['modules'])
    if len(mods) < 2:
        raise ValueError(f'{path}: モジュールは2つ以上要る（順運動学が解けない）')
    rng = d.get('steer_range')
    if rng is not None and float(rng) < math.pi / 2:
        # 反転と合わせても、届かない向きが残る（±90° あればどの向きへも行ける）
        raise ValueError(f'{path}: steer_range {rng} は π/2 以上にする')
    p = Params(modules=mods,
               wheel_speed_max=float(d['wheel_speed_max']),
               steer_rate_max=float(d['steer_rate_max']),
               steer_range=None if rng is None else float(rng),
               speed_max=float(d['speed_max']), accel_max=float(d['accel_max']),
               omega_max=float(d['omega_max']), alpha_max=float(d['alpha_max']),
               wheel_stop=float(d.get('wheel_stop', 1e-3)),
               steer_early_speed=float(d.get('steer_early_speed', 0.3)),
               steer_early_room=float(d.get('steer_early_room', math.radians(30))),
               steer_early_room_drift=float(d.get('steer_early_room_drift', math.radians(75))))
    if p.steer_early_speed < 0:
        raise ValueError(f'{path}: steer_early_speed は 0 以上にする')
    for k in ('steer_early_room', 'steer_early_room_drift'):
        if not 0 <= getattr(p, k) <= math.pi:
            raise ValueError(f'{path}: {k} は 0〜π にする')
    for k in ('wheel_speed_max', 'steer_rate_max', 'speed_max', 'accel_max',
              'omega_max', 'alpha_max'):
        if getattr(p, k) <= 0:
            raise ValueError(f'{path}: {k} は正の値にする')
    return p


def wrap(a):
    """角度を −π〜π [rad] に丸める."""
    return (np.asarray(a, dtype=float) + math.pi) % (2 * math.pi) - math.pi


# ── 座標の回転 ─────────────────────────────────────────────────────
def to_body(vx, vy, th, m=NP):
    """地図座標の速度を機体座標へ変換する（R(θ)ᵀ）."""
    c, s = m.cos(th), m.sin(th)
    return c * vx + s * vy, -s * vx + c * vy


def to_map(vbx, vby, th, m=NP):
    """機体座標の速度を地図座標へ変換する（R(θ)）."""
    c, s = m.cos(th), m.sin(th)
    return c * vbx - s * vby, s * vbx + c * vby


# ── 状態方程式 ─────────────────────────────────────────────────────
def deriv(z, u, n_steer=0):
    """
    状態方程式（二重積分器 6状態＋ステア角 n_steer 個）の時間微分リストを返す.

    z・u は添字アクセス可能な配列または CasADi ベクトル。
    """
    return ([z[3], z[4], z[5], u[0], u[1], u[2]]
            + [u[N_ACCEL + i] for i in range(n_steer)])


def rk4(z, u, dt, n_steer=0):
    """RK4 法で NumPy 配列の状態 `z` を `dt` [s] 進める."""
    z = np.asarray(z, dtype=float)
    u = np.asarray(u, dtype=float)

    def f(zz):
        return np.array(deriv(zz, u, n_steer), dtype=float)
    k1 = f(z)
    k2 = f(z + dt / 2 * k1)
    k3 = f(z + dt / 2 * k2)
    k4 = f(z + dt * k3)
    return z + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


# ── 運動学 ────────────────────────────────────────────────────────
def module_velocity(vbx, vby, om, r):
    """モジュールの機体座標速度 `(vbx − ω*ry, vby + ω*rx)` [m/s] を返す."""
    return vbx - om * r[1], vby + om * r[0]


def module_velocities(vbx, vby, om, p):
    return [module_velocity(vbx, vby, om, r) for r in p.modules]


def ik(vbx, vby, om, p, m=NP):
    """
    理想逆運動学 `(速さ [m/s], ステア角 [rad])` リストを返す（反転なし）.

    静止時は角度不定のため実指令には `ik_command` を用いる。
    """
    return [(m.sqrt(wx * wx + wy * wy), m.atan2(wy, wx))
            for wx, wy in module_velocities(vbx, vby, om, p)]


def steer_room(delta, vbx, vby, om, r, p):
    """
    ステア角 `delta` から可動端までの余裕 `[rad]` と流れ方向判定フラグを返す.

    地図上で同方向へ進みながら旋回する際はタイヤが `-ω` 側へ流れるためその方向の端を測る。
    """
    if abs(om) < OM_DRIFT or math.hypot(vbx, vby) <= abs(om) * math.hypot(*r):
        return p.steer_range - abs(delta), False
    return (p.steer_range + delta if om > 0 else p.steer_range - delta), True


def pick_side(i, cands, vbx, vby, om, r, p, near):
    """
    範囲付きステアで走行中に対角2輪（0・2番）を端到達前に先行反転させる.

    4輪同時反転による加減速喪失と壁際行き過ぎを防ぐため、偶数輪だけ先に振り直す。
    """
    if (i % 2 or len(cands) < 2 or p.steer_early_speed <= 0
            or math.hypot(vbx, vby) < p.steer_early_speed):
        return near
    room, drift = steer_room(near[0], vbx, vby, om, r, p)
    if room >= (p.steer_early_room_drift if drift else p.steer_early_room):
        return near
    return next(c for c in cands if c is not near)


def ik_targets(vbx, vby, om, delta_now, p):
    """
    モジュールの目標リスト `[(目標ステア角, 周速, 反転有無), ...]` を返す（cos 補正前）.

    - 90° 超の差は 180° 反転＋負速度で最短化（`steer_range` 外は除外）
    - `steer_early_speed` 以上では対角2輪を先行振替する
    - `wheel_stop` 未満の停止時は現在ステア角を維持し速度 0 を出す
    """
    out = []
    vels = module_velocities(vbx, vby, om, p)
    for i, (r, (wx, wy), d_now) in enumerate(zip(p.modules, vels, delta_now)):
        d_now = float(d_now)
        s = math.hypot(wx, wy)
        if s < p.wheel_stop:
            out.append((d_now, 0.0, False))
            continue
        a = math.atan2(wy, wx)
        cands = [(a, s, False), (float(wrap(a + math.pi)), -s, True)]
        if p.steer_range is not None:
            cands = [c for c in cands if abs(c[0]) <= p.steer_range + 1e-12] or cands
        # 今の角からいちばん近い向き。同じなら反転しない方（真横で回し始めるとき）
        near = min(cands, key=lambda c: (abs(float(wrap(c[0] - d_now))), c[2]))
        chosen = (pick_side(i, cands, vbx, vby, om, r, p, near)
                  if p.steer_range is not None else near)
        out.append(chosen)
    return out


def ik_command(vbx, vby, om, delta_now, p):
    """
    指令速度から cos 補正付き実駆動指令 `[(目標角, 補正済み周速, 反転有無), ...]` を返す.

    ステアが回り切る前に狙い外の方向へ推力が出ないよう `cos(目標 - 現在)` を掛ける。
    """
    return [(t, s * math.cos(float(wrap(t - float(d)))), f)
            for (t, s, f), d in zip(ik_targets(vbx, vby, om, delta_now, p), delta_now)]


def fk(speeds, deltas, p):
    """
    各モジュールの `(周速, ステア角)` から最小二乗で `(vbx, vby, om, 残差 [m/s])` を求める.

    残差が大きい場合はスリップまたはステア角の追従遅れを示す。
    """
    rows, rhs = [], []
    for (rx, ry), s, d in zip(p.modules, speeds, deltas):
        rows += [[1.0, 0.0, -ry], [0.0, 1.0, rx]]
        rhs += [s * math.cos(d), s * math.sin(d)]
    a, b = np.array(rows), np.array(rhs)
    x, *_ = np.linalg.lstsq(a, b, rcond=None)
    res = float(np.sqrt(np.mean((a @ x - b) ** 2)))
    return float(x[0]), float(x[1]), float(x[2]), res


def steer_toward(d_now, d_tgt, dt, p, rate=None):
    """
    ステアを最高角速度 `rate` [rad/s] で `dt` [s] 進めた後の角度 [rad] を返す.

    無制限時は最短経路、範囲制限時は `[-steer_range, steer_range]` 内で動かす。
    """
    lim = (p.steer_rate_max if rate is None else rate) * dt
    if p.steer_range is None:
        return float(wrap(d_now + np.clip(float(wrap(d_tgt - d_now)), -lim, lim)))
    tgt = float(np.clip(d_tgt, -p.steer_range, p.steer_range))
    return float(d_now + np.clip(tgt - d_now, -lim, lim))


# ── シムのモジュール模型（SW-47） ───────────────────────────────────
@dataclass(frozen=True)
class PlantCfg:
    """モジュール応答モデルの時定数・制限値（すべて 0 で理想動作）."""

    lag: float = 0.0            # 車輪の速さの一次遅れの時定数 [s]
    wheel_accel: float = 0.0    # 車輪の周速の変化の上限 [m/s^2]
    wheel_speed: float = 0.0    # 車輪の最高周速 [m/s]。超えたら全輪を同じ割合で縮める（向きを保つ）
    steer_rate: float = 0.0     # ステアの最高角速度 [rad/s]。0 は瞬時に向く

    @property
    def off(self):
        return not (self.lag or self.wheel_accel or self.wheel_speed or self.steer_rate)


def plant_step(deltas, speeds, cmd, dt, cfg, p):
    """
    モジュール模型を `dt` [s] 進めて `(ステア角列, 周速列, (vbx, vby, om))` を返す.

    `ik_targets -> ステア遅れ -> cos補正 -> 周速飽和 -> 一次遅れ -> 加速度制限 -> fk` を順に適用する。
    """
    tg = ik_targets(cmd[0], cmd[1], cmd[2], deltas, p)
    new_d = [steer_toward(d, t, dt, p, cfg.steer_rate) if cfg.steer_rate > 0 else t
             for (t, _, _), d in zip(tg, deltas)]
    want = [s * math.cos(float(wrap(t - d))) for (t, s, _), d in zip(tg, new_d)]
    if cfg.wheel_speed > 0:
        top = max(abs(w) for w in want)
        if top > cfg.wheel_speed:
            want = [w * cfg.wheel_speed / top for w in want]
    keep = math.exp(-dt / cfg.lag) if cfg.lag > 0 else 0.0
    new_s = []
    for w, s in zip(want, speeds):
        nxt = w + (s - w) * keep                 # 一次遅れ（厳密に離散化）
        if cfg.wheel_accel > 0:
            lim = cfg.wheel_accel * dt
            nxt = s + min(max(nxt - s, -lim), lim)
        new_s.append(nxt)
    vbx, vby, om, _ = fk(new_s, new_d, p)
    return new_d, new_s, (vbx, vby, om)


# ── 上限 ──────────────────────────────────────────────────────────
def max_speed(heading, om, p):
    """
    機体座標の方位 `heading` [rad]・角速度 `om` [rad/s] での最大並進速さ [m/s] を返す.

    全モジュール周速・機体速度ノルム・角速度上限から成立区間を計算し、不能時は None。
    """
    if abs(om) > p.omega_max + 1e-12:
        return None
    dx, dy = math.cos(heading), math.sin(heading)
    lo, hi = 0.0, p.speed_max
    for rx, ry in p.modules:
        b = om * (-dx * ry + dy * rx)
        c = om * om * (rx * rx + ry * ry) - p.wheel_speed_max ** 2
        disc = b * b - c
        if disc < 0:
            return None
        root = math.sqrt(disc)
        lo, hi = max(lo, -b - root), min(hi, -b + root)
    return hi if hi >= lo - 1e-12 else None


# ── 制約 ──────────────────────────────────────────────────────────
def state_constraints(z, p, m=NP, steer=True):
    """
    状態ベクトル `z` の制約一覧 `[(名前, 式, 下限, 上限), ...]` を返す.

    `z = [x, y, th, vx, vy, om, delta0...]`。`steer=False` 時はタイヤ関連を省く。
    """
    vx, vy, om, th = z[3], z[4], z[5], z[2]
    out = [('速度ノルム', vx * vx + vy * vy, None, p.speed_max ** 2),
           ('角速度', om, -p.omega_max, p.omega_max)]
    vbx, vby = to_body(vx, vy, th, m)
    for i, (wx, wy) in enumerate(module_velocities(vbx, vby, om, p)):
        out.append((f'車輪の周速 {i}', wx * wx + wy * wy, None, p.wheel_speed_max ** 2))
        if steer:
            d = z[N_POINT + i]
            # タイヤは向いている方向にしか転がらない（横滑りしない）。前後はどちらでもよい
            out.append((f'タイヤの向き {i}', wx * m.sin(d) - wy * m.cos(d), 0.0, 0.0))
            if p.steer_range is not None:
                out.append((f'ステア角の範囲 {i}', d, -p.steer_range, p.steer_range))
    return out


def input_constraints(u, p, steer=True):
    """入力ベクトル `u = [ax, ay, alpha, d_delta0...]` の制約一覧を返す."""
    out = [('加速度ノルム', u[0] * u[0] + u[1] * u[1], None, p.accel_max ** 2),
           ('角加速度', u[2], -p.alpha_max, p.alpha_max)]
    if steer:
        out += [(f'ステアの速さ {i}', u[N_ACCEL + i], -p.steer_rate_max, p.steer_rate_max)
                for i in range(p.n)]
    return out


def violations(z, u=None, p=None, tol=1e-6, steer=True):
    """数値評価で許容 `tol` を超えて違反している制約 `[(名前, 値, 下限, 上限), ...]` を返す."""
    rows = state_constraints(z, p, NP, steer)
    if u is not None:
        rows += input_constraints(u, p, steer)
    bad = []
    for name, val, lo, hi in rows:
        v = float(val)
        if (lo is not None and v < lo - tol) or (hi is not None and v > hi + tol):
            bad.append((name, v, lo, hi))
    return bad


def main():
    ap = argparse.ArgumentParser(description='4輪独立ステアの値と上限の目安を出す（SW-44）')
    ap.add_argument('--params', default=None, help=f'YAML（既定 {DEFAULT_PATH}）')
    args = ap.parse_args()
    p = load(args.params)
    print(f'値：{args.params or DEFAULT_PATH}（どれも仮・未確認）')
    print(f'  モジュール {p.modules} m')
    rng_str = '制限なし' if p.steer_range is None else f'±{math.degrees(p.steer_range):.0f}°'
    print(f'  車輪の周速 {p.wheel_speed_max} m/s・ステア {p.steer_rate_max} rad/s・'
          f'ステア角の範囲 {rng_str}')
    print(f'  速度 {p.speed_max} m/s・加速度 {p.accel_max} m/s²・角速度 {p.omega_max} rad/s・'
          f'角加速度 {p.alpha_max} rad/s²')
    print('並進の最高速 [m/s]（向き 0°／45°／90°）')
    for om in (0.0, 0.5, 1.0, 1.5):
        vs = [max_speed(math.radians(h), om, p) for h in (0, 45, 90)]
        txt = '／'.join('—' if v is None else f'{v:.3f}' for v in vs)
        print(f'  ω = {om:.1f} rad/s：{txt}')
    print(f'タイヤを 90° 回す時間 {math.pi / 2 / p.steer_rate_max:.2f} s')


if __name__ == '__main__':
    main()
