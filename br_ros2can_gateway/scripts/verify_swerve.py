#!/usr/bin/env python3
"""
4輪独立ステアの式（`swerve.py`、SW-44）の単体試験（シム・ROS 不要）.

    ros2 run br_ros2can_gateway verify_swerve.py
    pytest br_ros2can_gateway/scripts/verify_swerve.py -q
    （CasADi が入っている Python で流すと、記号の式も確かめる）

SW-44 完了条件（往復・円上限・接線・回転並進・90°反転）に加え、
cos 補正・停止保持・サーボ角範囲・制約判定・YAML・状態方程式を検証する。
"""

import math
import os
import re
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import swerve as S                                                 # noqa: E402

P = S.load()
RNG = np.random.default_rng(20261007)
TESTS = []
_NOTE = []


def case(name):
    """試験を登録する飾り（pytest からは `test_*` として直接呼ばれる）."""
    def deco(fn):
        TESTS.append((name, fn))
        return fn
    return deco


def note(text):
    """表に出す一言を残す（pytest 警告を避けるため試験関数は値を返さない）."""
    _NOTE.append(text)


def deg(a):
    return math.degrees(float(a))


@case('逆 → 順の往復で機体速度が一致する（反転した表し方でも同じ）')
def test_roundtrip():
    worst = 0.0
    for _ in range(500):
        vbx, vby = RNG.uniform(-0.5, 0.5, 2)
        om = RNG.uniform(-1.5, 1.5)
        mods = S.ik(vbx, vby, om, P)
        got = S.fk([s for s, _ in mods], [d for _, d in mods], P)
        flip = S.fk([-s for s, _ in mods], [d + math.pi for _, d in mods], P)
        for g in (got, flip):
            worst = max(worst, abs(g[0] - vbx), abs(g[1] - vby), abs(g[2] - om), g[3])
    assert worst < 1e-9, worst
    note(f'500 通り・最大の差 {worst:.1e}')


@case('並進だけなら上限は半径＝車輪の周速の円（斜めでも遅くならない）')
def test_translation_circle():
    vs = [S.max_speed(math.radians(h), 0.0, P) for h in range(0, 360, 5)]
    want = min(P.wheel_speed_max, P.speed_max)
    assert all(v is not None and abs(v - want) < 1e-12 for v in vs), vs
    # 斜め 45° で車輪の周速がちょうど上限（四角で抑えると斜めは √2 倍出てしまう）
    mods = S.ik(want * math.cos(math.pi / 4), want * math.sin(math.pi / 4), 0.0, P)
    assert all(abs(float(s) - P.wheel_speed_max) < 1e-12 for s, _ in mods)
    note(f'72 方向とも {want:.3f} m/s（いまのシムは斜めで 0.55〜0.64 m/s 出る）')


@case('その場回転で4輪が回転の接線方向を向く')
def test_spin_tangent():
    worst = 0.0
    for om in (0.5, -1.0, 1.5):
        for (s, d), (rx, ry) in zip(S.ik(0.0, 0.0, om, P), P.modules):
            worst = max(worst, abs(math.cos(d) * rx + math.sin(d) * ry))   # 半径と直交
            assert abs(float(s) - abs(om) * math.hypot(rx, ry)) < 1e-12
    angles = [round(deg(d)) for _, d in S.ik(0.0, 0.0, 1.0, P)]
    assert worst < 1e-12, worst
    note(f'ω = 1 rad/s のステア角 {angles}°（半径との内積の最大 {worst:.1e}）')


@case('回しながら進むと並進の上限が下がる（|ω| が大きいほど下がる）')
def test_rotation_lowers():
    rows = []
    for h in (0, 30, 45, 90, 180):
        prev = S.max_speed(math.radians(h), 0.0, P)
        for om in (0.5, 1.0, 1.5):
            v = S.max_speed(math.radians(h), om, P)
            assert v is not None and v < prev - 1e-9, (h, om, v, prev)
            prev = v
        rows.append(prev)
    v1 = S.max_speed(0.0, 1.0, P)
    note(f'前へ：ω 0 → 1.0 rad/s で {S.max_speed(0.0, 0.0, P):.3f} → {v1:.3f} m/s')


@case('目標のステア角が 90° を超えて離れると反転する（ステアを 180° ずらし、車輪を逆に回す）')
def test_flip():
    now = [0.0] * P.n
    back = S.ik_command(-0.3, 0.3, 0.0, now, P)            # 斜め後ろ（135°）へ
    assert all(f for _, _, f in back), back
    assert all(abs(deg(t) + 45) < 1e-9 and s < 0 for t, s, _ in back), back
    side = S.ik_command(0.3 * math.cos(math.radians(80)), 0.3 * math.sin(math.radians(80)),
                        0.0, now, P)                      # 80° なら反転しない
    assert not any(f for _, _, f in side), side
    rev = S.ik_command(-0.3, 0.0, 0.0, now, P)            # 真後ろはステアを回さず逆回し
    assert all(abs(t) < 1e-12 and abs(s + 0.3) < 1e-12 for t, s, _ in rev), rev
    note('135° は −45° へ逆回し、80° はそのまま、180° は回さず逆回し')


@case('cos 補正：止まった所から真横へ指令しても、タイヤが回り切るまではほとんど進まない')
def test_cos_correction():
    now = [0.0] * P.n
    side = S.ik_command(0.0, 0.3, 0.0, now, P)           # 前向きのまま真横へ
    assert all(abs(s) < 1e-12 for _, s, _ in side), side
    half = S.ik_command(0.0, 0.3, 0.0, [math.radians(45)] * P.n, P)   # 半分回った所
    assert all(abs(s - 0.3 * math.cos(math.radians(45))) < 1e-12 for _, s, _ in half), half
    note('0° で車輪 0 m/s、45° まで回ると 0.212 m/s（0.3 × cos 45°）')


@case('車輪の速さがほぼ 0 ならステア角を今のまま保つ')
def test_hold():
    now = [0.3, -0.2, 1.0, -2.5]
    cmd = S.ik_command(0.0, 0.0, 0.0, now, P)
    assert [t for t, _, _ in cmd] == now and all(s == 0.0 for _, s, _ in cmd), cmd
    note(f'指令 0 で {len(now)} 輪とも角を保つ')


@case('サーボ型（±135°）では、範囲の外へステアを向けない')
def test_servo_range():
    q = S.load(steer_range=math.radians(135))
    worst = 0.0
    for _ in range(500):
        now = list(RNG.uniform(-q.steer_range, q.steer_range, q.n))
        vbx, vby = RNG.uniform(-0.5, 0.5, 2)
        om = RNG.uniform(-1.5, 1.5)
        for t, _, _ in S.ik_command(vbx, vby, om, now, q):
            worst = max(worst, abs(t))
        d = now[0]
        for t in (2.0, -2.5, 3.1):
            d = S.steer_toward(d, t, 1.0, q)
            worst = max(worst, abs(d))
    assert worst <= q.steer_range + 1e-12, deg(worst)
    note(f'500 通りで最大 {deg(worst):.1f}°')


@case('範囲つき：走っている間は、対角の2輪（0・2）を端の手前で先に振り直し、残りの2輪は今の向きを保つ')
def test_early_flip():
    q = S.load(steer_range=math.radians(135), steer_early_speed=0.3)
    a = math.radians(120)                                  # 端まで 15°
    tg = S.ik_targets(0.6 * math.cos(a), 0.6 * math.sin(a), 0.0, [a] * q.n, q)
    assert all(abs(deg(tg[i][0]) + 60) < 1e-9 and tg[i][1] < 0 and tg[i][2] for i in (0, 2)), tg
    assert all(abs(deg(tg[i][0]) - 120) < 1e-9 and tg[i][1] > 0 and not tg[i][2]
               for i in (1, 3)), tg
    note('120° へ 0.6 m/s：0・2 は −60° へ逆回し、1・3 は 120° のまま')


@case('範囲つき：遅い間・端まで余裕があるとき・先に振り直さない設定では、今の向きを保つ')
def test_early_flip_keep():
    q = S.load(steer_range=math.radians(135), steer_early_speed=0.3)
    off = S.load(steer_range=math.radians(135), steer_early_speed=0.0)
    for a, v, r in ((120, 0.2, q), (90, 0.6, q), (120, 0.6, off)):
        d = math.radians(a)
        tg = S.ik_targets(v * math.cos(d), v * math.sin(d), 0.0, [d] * q.n, r)
        assert not any(f for _, _, f in tg), (a, v, tg)
    d = math.radians(80)
    # 右回りは + 側の端へ流れる / 左回りは - 側の端へ流れる
    assert abs(deg(S.steer_room(d, 0.6, 0.0, -1.0, q.modules[0], q)[0]) - 55) < 1e-9
    assert abs(deg(S.steer_room(d, 0.6, 0.0, 1.0, q.modules[0], q)[0]) - 215) < 1e-9
    note('0.2 m/s・端まで 45°・設定 0 は振り直さない。機体の回る向きで端までの余裕が変わる')


@case('制限なしでは、先に振り直さない')
def test_early_flip_free():
    q = S.load(steer_range=None, steer_early_speed=0.3)
    a = math.radians(120)
    tg = S.ik_targets(0.6 * math.cos(a), 0.6 * math.sin(a), 0.0, [a] * q.n, q)
    assert not any(f for _, _, f in tg), tg
    note('120° へ 0.6 m/s でも4輪とも今の向き')


@case('ステアは最高角速度で、近い回り方で目標へ向かう（制限なし）')
def test_steer_toward():
    # ここは制限なしの形を見るので、YAML の steer_range がどちらでも範囲を外して読む
    q = S.load(steer_range=None)
    d = S.steer_toward(math.radians(170), math.radians(-170), 0.01, q)   # +20° 回せば着く
    assert abs(deg(d) - (170 + math.degrees(q.steer_rate_max * 0.01))) < 1e-9, deg(d)
    t90 = 0.0
    d = 0.0
    while abs(d - math.pi / 2) > 1e-9:
        d = S.steer_toward(d, math.pi / 2, 0.001, q)
        t90 += 0.001
    assert abs(t90 - math.pi / 2 / q.steer_rate_max) < 2e-3, t90
    note(f'170° → −170° は +側へ。90° 回すのに {t90:.3f} s')


@case('制約の判定：守る状態は通し、破る状態はその名前で返す')
def test_constraints():
    v = P.speed_max
    straight = [0, 0, 0.3, v * math.cos(0.3), v * math.sin(0.3), 0.0] + [0.0] * P.n
    assert S.violations(straight, [0.0, 0.0, 0.0] + [0.0] * P.n, P) == []

    def names(bad):
        return {n.split(' ')[0] for n, *_ in bad}

    skew = list(straight)
    skew[S.N_POINT] = 0.5                                  # 1輪だけ向きがずれている
    assert names(S.violations(skew, None, P)) == {'タイヤの向き'}
    fast = [0, 0, 0, 1.2 * v, 0.0, 0.0] + [0.0] * P.n      # 上限の 1.2 倍
    assert {'速度ノルム', '車輪の周速'} <= names(S.violations(fast, None, P))
    spin = [0, 0, 0, v, 0.0, 1.0] + [0.0] * P.n            # 前へ全速で回す → 外側の車輪が超える
    assert '車輪の周速' in names(S.violations(spin, None, P, steer=False))
    acc = S.violations(straight, [1.2, 1.2, 4.0, 3.5, 0, 0, 0], P)
    assert {'加速度ノルム', '角加速度', 'ステアの速さ'} <= names(acc), acc
    note('タイヤの向き・速度ノルム・車輪の周速・加速度ノルム・角加速度・ステアの速さを検出')


@case('値は YAML だけで変わる（知らないキーは断る）')
def test_yaml():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'swerve.yaml')
        with open(S.DEFAULT_PATH, encoding='utf-8') as f:
            text = f.read()
        with open(path, 'w', encoding='utf-8') as f:
            # 置き換えるのは今の値の行（2026-10-29 に 0.5 → 2.0 にしたら、決め打ちの 0.5 で置き換わらず落ちた）
            updated = re.sub(r'^wheel_speed_max: .*$', 'wheel_speed_max: 0.8', text, flags=re.M)
            f.write(updated.replace('steer_range: null', 'steer_range: 2.356'))
        q = S.load(path)
        assert q.wheel_speed_max == 0.8 and abs(q.steer_range - 2.356) < 1e-12
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text + '\nwheel_speeed_max: 1.0\n')
        try:
            S.load(path)
        except ValueError:
            pass
        else:
            raise AssertionError('綴りを間違えたキーを通した')
    note('周速とステア角の範囲を YAML で替えられ、綴りの間違いは断る')


@case('状態方程式：加速度一定なら等加速度の式と一致する（RK4）')
def test_deriv():
    z = np.array([0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    u = np.array([1.0, -0.5, 0.8, 1.0, 0.0, -1.0, 2.0])
    t, dt = 0.0, 0.05
    while t < 1.0 - 1e-12:
        z = S.rk4(z, u, dt, n_steer=4)
        t += dt
    want = [0.1 + 0.5, -0.25, 0.4, 1.1, -0.5, 0.8, 1.0, 0.0, -1.0, 2.0]
    assert np.allclose(z, want, atol=1e-12), z
    note('1 s 後の位置・向き・速度・ステア角が式どおり')


@case('CasADi の記号でも同じ式が通る（.venv の python のときだけ）')
def test_casadi():
    try:
        import casadi as ca
    except ImportError:
        note('SKIP（CasADi が無い。.venv/bin/python で流すと見る）')
        return
    m = S.backend(ca)
    z = ca.SX.sym('z', S.N_POINT + P.n)
    u = ca.SX.sym('u', S.N_ACCEL + P.n)
    exprs = [e for _, e, _, _ in S.state_constraints(z, P, m)]
    exprs += [e for _, e, _, _ in S.input_constraints(u, P)]
    exprs += S.deriv(z, u, n_steer=P.n)
    f = ca.Function('f', [z, u], [ca.vertcat(*exprs)])
    worst = 0.0
    for _ in range(50):
        zv = RNG.uniform(-1, 1, S.N_POINT + P.n)
        uv = RNG.uniform(-1, 1, S.N_ACCEL + P.n)
        want = [e for _, e, _, _ in S.state_constraints(zv, P)]
        want += [e for _, e, _, _ in S.input_constraints(uv, P)]
        want += S.deriv(zv, uv, n_steer=P.n)
        got = np.array(f(zv, uv)).ravel()
        worst = max(worst, float(np.max(np.abs(got - np.array(want, dtype=float)))))
    assert worst < 1e-12, worst
    n_con = len(exprs) - S.N_POINT - P.n
    note(f'制約 {n_con} 本と状態方程式で numpy との差 {worst:.1e}（casadi {ca.__version__}）')


def simulate(cmd, secs, cfg, dt=0.01, deltas=None):
    """模型を一定指令で `secs` [s] 回し `(時刻, 機体速度, (x, y))` の列を返す."""
    d = list(deltas if deltas is not None else [0.0] * P.n)
    s = [0.0] * P.n
    x = y = th = 0.0
    out = []
    for k in range(int(round(secs / dt))):
        d, s, vb = S.plant_step(d, s, cmd, dt, cfg, P)
        vx, vy = S.to_map(vb[0], vb[1], th)
        x, y, th = x + vx * dt, y + vy * dt, th + vb[2] * dt
        out.append(((k + 1) * dt, vb, (x, y)))
    return out


@case('模型（SW-47）：値が全部 0 なら、指令した機体速度がそのまま出る')
def test_plant_off():
    cfg = S.PlantCfg()
    assert cfg.off
    worst = 0.0
    for _ in range(300):
        cmd = (*RNG.uniform(-0.5, 0.5, 2), RNG.uniform(-1.5, 1.5))
        d0 = list(RNG.uniform(-math.pi, math.pi, P.n))
        _, _, vb = S.plant_step(d0, [0.0] * P.n, cmd, 0.01, cfg, P)
        worst = max(worst, *(abs(a - b) for a, b in zip(vb, cmd)))
    assert worst < 1e-9, worst
    note(f'300 通り・最大の差 {worst:.1e}（どのステア角から始めても同じ）')


@case('模型：一次遅れのステップ応答が時定数どおり（τ 後に 63 %）')
def test_plant_lag():
    tau = 0.2
    run_ = simulate((0.3, 0.0, 0.0), 3 * tau, S.PlantCfg(lag=tau))
    at = {round(t, 2): vb[0] for t, vb, _ in run_}
    want1, want3 = 0.3 * (1 - math.exp(-1)), 0.3 * (1 - math.exp(-3))
    assert abs(at[0.2] - want1) < 1e-6 and abs(at[0.6] - want3) < 1e-6, (at[0.2], at[0.6])
    note(f'τ = {tau} s：{at[0.2]:.4f} m/s（式 {want1:.4f}）、3τ で {at[0.6]:.4f}（式 {want3:.4f}）')


@case('模型：車輪の加速度の上限が効く')
def test_plant_accel():
    run_ = simulate((0.5, 0.0, 0.0), 0.6, S.PlantCfg(wheel_accel=1.0))
    at = {round(t, 2): vb[0] for t, vb, _ in run_}
    assert abs(at[0.25] - 0.25) < 1e-9 and abs(at[0.5] - 0.5) < 1e-9, (at[0.25], at[0.5])
    note('1.0 m/s² で 0.25 s 後 0.25 m/s、0.5 s 後に 0.5 m/s へ届く')


@case('模型：止まった所から真横へ指令すると、タイヤが回り切るまではほとんど進まない（cos 補正）')
def test_plant_sideways():
    cfg = S.PlantCfg(steer_rate=3.0)
    run_ = simulate((0.0, 0.3, 0.0), 0.8, cfg)
    t_turn = math.pi / 2 / cfg.steer_rate
    early = [math.hypot(vb[0], vb[1]) for t, vb, _ in run_ if t <= 0.05 + 1e-9]
    late = [vb for t, vb, _ in run_ if t >= t_turn + 0.02]
    x_turn, y_turn = next(p for t, _, p in run_ if t >= t_turn - 1e-9)
    assert max(early) < 0.05, early                    # 指令 0.3 m/s に対して最初の 0.05 s
    assert all(abs(vb[1] - 0.3) < 1e-9 and abs(vb[0]) < 1e-9 for vb in late)
    # 回している間は、車輪がその時の向きへ 0.3·cos(90° − δ) で転がる：真横 0.3·π/12、前へ 0.05 m
    assert abs(y_turn - 0.3 * math.pi / 12) < 0.005 and abs(x_turn - 0.05) < 0.005, (
        x_turn, y_turn)
    note(f'最初の 0.05 s は {max(early):.3f} m/s 以下。回り切る {t_turn:.2f} s までに真横 {y_turn:.3f} m・'
         f'前へ {x_turn:.3f} m（理想なら真横 {0.3 * t_turn:.3f} m）')


@case('模型：周速が上限を超えたら全輪を同じ割合で縮める（進む向きを保つ）')
def test_plant_saturation():
    cmd = (0.5, 0.0, 1.0)
    d, s, vb = S.plant_step([0.0] * P.n, [0.0] * P.n, cmd, 0.01, S.PlantCfg(wheel_speed=0.5), P)
    assert max(abs(v) for v in s) <= 0.5 + 1e-12, s
    assert abs(vb[0] / vb[2] - cmd[0] / cmd[2]) < 1e-9 and abs(vb[1]) < 1e-9, vb
    note(f'前へ 0.5 m/s＋1.0 rad/s の指令 → 機体 {vb[0]:.3f} m/s・{vb[2]:.3f} rad/s（比は同じ）')


@case('模型：真後ろへの指令はステアを回さず逆回しで出る（反転）')
def test_plant_reverse():
    d, s, vb = S.plant_step([0.0] * P.n, [0.0] * P.n, (-0.3, 0.0, 0.0), 0.01,
                            S.PlantCfg(steer_rate=3.0), P)
    assert all(abs(x) < 1e-12 for x in d) and abs(vb[0] + 0.3) < 1e-9, (d, vb)
    note('ステア角 0 のまま、1 周期目から −0.3 m/s')


def run():
    bad = []
    for name, fn in TESTS:
        try:
            _NOTE.clear()
            fn()
            text = _NOTE[-1] if _NOTE else ''
            mark = 'SKIP' if text.startswith('SKIP') else 'OK  '
            print(f'  {mark} {name}' + (f'：{text}' if text else ''))
        except AssertionError as e:
            print(f'  NG   {name}：{e}')
            bad.append(name)
    return bad


if __name__ == '__main__':
    print(f'値：{S.DEFAULT_PATH}（仮・未確認）')
    _bad = run()
    print(f"\n{'合格' if not _bad else '不合格'} {len(TESTS)} 項目 / 指摘 {len(_bad)} 件")
    sys.exit(1 if _bad else 0)
