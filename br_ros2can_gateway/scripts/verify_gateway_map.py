#!/usr/bin/env python3
"""
仲介ノードの割り当て表と換算の単体試験（SW-65）。ROS もシムも要らない.

使い方:
  python3 br_ros2can_gateway/scripts/verify_gateway_map.py
  ros2 run br_ros2can_gateway verify_gateway_map.py
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import gateway_map as G                                            # noqa: E402, I100

M = G.load()
RNG = random.Random(20261115)
TESTS = []
_NOTE = []
ZEROS = [0.0] * M.n


def case(name):
    """試験ケースを登録するデコレータ."""
    def deco(fn):
        TESTS.append((name, fn))
        return fn
    return deco


def note(text):
    """合格メッセージの補足を記録する."""
    _NOTE.append(text)


def refuses(**override):
    """指定の override で G.load が ValueError で拒否することを確かめる."""
    try:
        G.load(**override)
    except ValueError as e:
        return str(e)
    raise AssertionError(f'断らなかった：{override}')


def yaml_with(**changes):
    """既定の表の一部を替えた dict を返す（load の override に渡す）."""
    import yaml
    with open(G.DEFAULT_PATH, encoding='utf-8') as f:
        d = yaml.safe_load(f)
    for key, val in changes.items():
        top, _, sub = key.partition('.')
        if sub:
            d[top] = {**d[top], sub: val}
        else:
            d[top] = val
    return {k: d[k] for k in {c.partition('.')[0] for c in changes}}


@case('割り当て表が読め、仮の値が残っている間は実機の形を断る')
def test_load():
    """割り当て表読込と require_real の検証."""
    assert M.n == 4 and M.rate_hz >= 50.0
    assert M.provisional, '仮の値の一覧が空（実機で確かめる前に空にしない）'
    try:
        G.require_real(M)
    except ValueError as e:
        msg = str(e)
    else:
        raise AssertionError('仮の値が残っているのに通した')
    G.require_real(G.load(provisional=[]))
    note(f'{M.n} モジュール・仮の値 {len(M.provisional)} 件（{msg[:24]}…）')


@case('スロットの並び：モータ n は n × 6 から、デバイスごとに 24 個、ステアと駆動は別のデバイス')
def test_slots():
    """TX スロット配置の検証."""
    fr, flags = G.encode_run(M, [0.1, 0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8], [0.0] * 4, ZEROS)
    assert set(fr) == {M.dev_steer, M.dev_drive}
    assert all(len(v) == G.N_SLOTS for v in fr.values())
    assert not flags, flags
    for i in range(4):
        s, d = fr[M.dev_steer][i * 6:i * 6 + 6], fr[M.dev_drive][i * 6:i * 6 + 6]
        assert s[0] == G.MODE_POS and s[2] == 3000 and s[3:] == [0, 0, 0], s
        assert d[0] == G.MODE_VEL and d[2:] == [0, 0, 0, 0], d
        assert s[1] == round((i + 1) * 0.1 * M.steer_gear / 0.001), s
        assert d[1] == round((0.5 + i * 0.1) / 0.04 * M.drive_gear / 0.01), d
    note(
        f'ステア → serial_tx_{M.dev_steer}（角度・速さの上限 30 rad/s）、'
        f'駆動 → serial_tx_{M.dev_drive}（速度）')


def roundtrip(m, deltas, speeds, pos, zeros):
    """指令のフレームを、そのまま帰還として読む（位置・速度のスロットの刻みは同じ）."""
    fr, flags = G.encode_run(m, deltas, speeds, pos, zeros)
    rx = {dev: [0] * G.N_SLOTS for dev in fr}
    for mot in m.steer:
        o = mot.index * 6
        rx[m.dev_steer][o + 3] = fr[m.dev_steer][o + 1]
    for mot in m.drive:
        o = mot.index * 6
        rx[m.dev_drive][o + 4] = fr[m.dev_drive][o + 1]
    dec = G.decode(m, rx)
    return G.module_state(m, dec, [r['pos'] for r in dec['steer']], zeros), flags


@case('物理量 → スロット → 物理量の往復が、刻みの半分以内で一致する')
def test_roundtrip():
    """エンコード・デコードの量子化誤差検証."""
    worst_d = worst_s = 0.0
    for _ in range(500):
        top = M.steer_pos_limit / M.steer_gear          # 逆運動学ノードが出すステア角は範囲の中
        deltas = [RNG.uniform(-top, top) for _ in range(4)]
        speeds = [RNG.uniform(-1.25, 1.25) for _ in range(4)]
        (d2, s2), flags = roundtrip(M, deltas, speeds, [0.0] * 4, ZEROS)
        assert not flags, flags
        worst_d = max(worst_d, max(abs(G.wrap(a - b)) for a, b in zip(deltas, d2)))
        worst_s = max(worst_s, max(abs(a - b) for a, b in zip(speeds, s2)))
    assert worst_d <= 0.0005 / M.steer_gear + 1e-12, worst_d
    assert worst_s <= 0.005 / M.drive_gear * M.wheel_radius + 1e-12, worst_s
    note(
        f'500 通り：ステア角の差は最大 {math.degrees(worst_d):.4f}°、'
        f'車輪の速さの差は最大 {worst_s * 1000:.3f} mm/s')


@case('向き（sign）を逆にすると、指令も帰還も逆になる')
def test_sign():
    """符号設定の対称性検証."""
    def rows(sign):
        return [{'module': i, 'index': i, 'sign': sign, 'home': 0.0} for i in range(4)]

    def drv(sign):
        return [{'module': i, 'index': i, 'sign': sign} for i in range(4)]

    q = G.load(motors={'steer': rows(-1), 'drive': drv(-1)})
    a, _ = G.encode_run(M, [0.3] * 4, [0.5] * 4, [0.0] * 4, ZEROS)
    b, _ = G.encode_run(q, [0.3] * 4, [0.5] * 4, [0.0] * 4, ZEROS)
    assert b[q.dev_steer][1] == -a[M.dev_steer][1] and b[q.dev_drive][1] == -a[M.dev_drive][1]
    (d2, s2), _ = roundtrip(q, [0.3] * 4, [0.5] * 4, [0.0] * 4, ZEROS)
    assert all(abs(x - 0.3) < 1e-3 for x in d2) and all(abs(x - 0.5) < 1e-3 for x in s2)
    note('sign −1 でスロットの符号が反転し、読み戻すと元の δ・s に戻る')


@case('駆動は上限 45.6 rad/s で切って印を立てる。シムの上限 1.25 m/s は切らない')
def test_drive_limit():
    """駆動速度飽和と clip フラグの検証."""
    top = M.drive_vel_max / M.drive_gear * M.wheel_radius
    fr, flags = G.encode_run(
        M, [0.0] * 4, [1.25, -1.25, top + 0.2, -(top + 0.2)], [0.0] * 4, ZEROS)
    d = fr[M.dev_drive]
    assert list(flags) == [(2, 'clip'), (3, 'clip')], flags
    assert d[13] == 4560 and d[19] == -4560 and abs(d[1]) < 4560 and d[1] == -d[7], d
    note(
        f'車輪の最高周速 {top:.3f} m/s。1.25 m/s は出力軸 {d[1] * 0.01:.2f} rad/s '
        'で上限の内側')


@case('暴走の見張り：逆向き・上限超えは 0.3 s で決め、ふつうの加減速と止まった車輪は数えない')
def test_runaway():
    """暴走監視 RunawayWatch の誤検知・検出時間検証."""
    dt = 1.0 / M.rate_hz

    def first_hit(cmds, vels):
        """関数 cmds・vels を 3 s 流して、最初に暴走と決めた時刻を返す（無ければ None）."""
        w = G.RunawayWatch(M)
        for k in range(int(3.0 / dt)):
            t = k * dt
            if w.step(t, [cmds(t)] + [0.0] * 3, [vels(t)] + [0.0] * 3) == 0:
                return t
        return None

    # ESC の ACC 2 の上限（BLDC 担当の計算、出力軸 104 rad/s^2）で指令へ寄る軌道
    def ramp(v0, v1, acc=104.0):
        return lambda t: v1 + (v0 - v1) * max(1.0 - acc * t / abs(v1 - v0), 0.0)

    t1 = first_hit(lambda t: 6.0, lambda t: -40.0)                 # 11/9 の形：+6 に -40
    assert t1 is not None and M.runaway_hold <= t1 <= M.runaway_hold + 3 * dt, t1
    assert first_hit(lambda t: 0.0, lambda t: -40.0) is not None   # 停止指令中に回転
    assert first_hit(lambda t: -10.0, lambda t: -40.0) is not None
    assert first_hit(lambda t: 20.0, lambda t: M.drive_vel_max + M.runaway_over) is not None
    assert first_hit(
        lambda t: M.drive_vel_max, lambda t: M.drive_vel_max + M.runaway_over - 0.1) is None
    assert first_hit(lambda t: -40.0, ramp(40.0, -40.0)) is None   # 全速反転（0.38 s）
    assert first_hit(lambda t: 0.0, ramp(45.6, 0.0)) is None       # 全速停止（0.44 s）
    assert first_hit(lambda t: 45.6, ramp(0.0, 45.6)) is None      # 停止から発進
    assert first_hit(lambda t: 20.0, lambda t: 0.0) is None        # 停止車輪は除外
    assert first_hit(lambda t: 6.0, lambda t: 6.0 - M.runaway_diff + 0.1) is None
    note(
        f'11/9 の形は {t1:.2f} s で決める。全速の反転・停止・発進は数えない'
        '（しきい値は BLDC 担当の例で仮）')


@case('ステアは近い回り方で目標へ向かい、範囲の端を越えるときだけ巻き戻して印を立てる')
def test_steer_range():
    """ステア回転範囲・巻き戻し・反転の検証."""
    g, mot = M.steer_gear, M.steer[0]
    wide = G.load(**yaml_with(**{'steer.pos_limit': 12.0}))
    # 範囲 ±12 rad：+170° から -170° への最短は +190°（範囲外）→ -170° へ巻き戻し
    tgt, flag = G.steer_target(wide, mot, math.radians(-170), math.radians(170) * g, 0.0)
    assert flag == 'unwind' and abs(tgt - math.radians(-170) * g) < 1e-9, (tgt, flag)
    tgt, flag = G.steer_target(wide, mot, math.radians(175), math.radians(100) * g, 0.0)
    assert flag is None and abs(tgt - math.radians(175) * g) < 1e-9
    tgt, flag = G.steer_target(M, mot, math.radians(120), math.radians(100) * g, 0.0)
    assert flag is None and abs(tgt - math.radians(120) * g) < 1e-9

    # 既定 ±135° で原点が 30° ずれ +135° を狙う場合は半周反転
    zero = math.radians(30) * g
    tgt, flag = G.steer_target(M, mot, math.radians(135), zero, zero)
    assert flag == 'flip' and abs(tgt) <= M.steer_pos_limit
    assert abs(G.wrap((tgt - zero) / g - math.radians(-45))) < 1e-9
    fr, fl = G.encode_run(M, [math.radians(135)] * 4, [0.5] * 4, [zero] * 4, [zero] * 4)
    assert (0, 'flip') in fl and all(fr[M.dev_drive][i * 6 + 1] < 0 for i in range(4))

    # 範囲なしでは +190° を通し、int16 スロット端で巻き戻す
    free = G.load(**yaml_with(**{'steer.pos_limit': None}))
    tgt, flag = G.steer_target(free, mot, math.radians(-170), math.radians(170) * g, 0.0)
    assert flag is None and abs(tgt - math.radians(190) * g) < 1e-9
    far = 32.7
    tgt, flag = G.steer_target(free, mot, G.wrap(far / g + 0.2), far, 0.0)
    assert flag == 'unwind' and abs(tgt) <= 32.767
    assert abs(G.wrap(tgt / g - G.wrap(far / g + 0.2))) < 1e-9

    for _ in range(2000):
        zero = RNG.uniform(-math.pi * g, math.pi * g)
        pos = RNG.uniform(-M.steer_pos_limit, M.steer_pos_limit)
        d = RNG.uniform(-math.pi, math.pi)
        tgt, flag = G.steer_target(M, mot, d, pos, zero)
        assert flag in (None, 'unwind', 'flip') and abs(tgt) <= M.steer_pos_limit + 1e-9
        expected = d + (math.pi if flag == 'flip' else 0.0)
        assert abs(G.wrap((tgt - zero) / g - expected)) < 1e-9
    note(
        f'±{M.steer_pos_limit:g} rad（ステア軸 ±{math.degrees(M.steer_pos_limit / g):.0f}°）'
        'の中で 2000 通り。範囲なしでは ±32.767 rad（スロットの端）で巻き戻す')


@case('止めるフレーム：脱力は全部 0、フリーズは駆動が速度 0・ステアが今の位置')
def test_stop_frames():
    """encode_relax と encode_hold の検証."""
    relax = G.encode_relax(M)
    assert all(v == [0] * G.N_SLOTS for v in relax.values())
    assert set(relax) == {M.dev_steer, M.dev_drive}
    pos = [1.0, -2.5, 11.9, 12.4]
    hold = G.encode_hold(M, pos)
    for i, p in enumerate(pos):
        s, d = hold[M.dev_steer][i * 6:i * 6 + 3], hold[M.dev_drive][i * 6:i * 6 + 2]
        assert s == [G.MODE_POS, round(min(p, M.steer_pos_limit) / 0.001), 3000], s
        assert d == [G.MODE_VEL, 0], d
    note('フリーズは脱力させない（dev_ws の決定事項60）。範囲の外にいたら端で保つ')


@case('位置の展開は幅 25 rad。幅 32768 個で展開すると、折り返すたびに 7.768 rad ずれる')
def test_unwrap():
    """ステア位置展開 unwrap と seed_total の検証."""
    total, truth, step = 0.0, 0.0, 0.7
    worst = 0.0
    for i in range(400):
        truth += step if i < 250 else -step
        raw = (truth + 12.5) % 25.0 - 12.5
        raw = round(raw / 0.001) * 0.001
        total = G.unwrap(total, raw)
        worst = max(worst, abs(total - truth))
    assert worst < 1e-3, worst
    wrong = 32.768 - 25.0
    assert abs(wrong - 7.768) < 1e-9
    assert abs(G.seed_total(-12.3, 87.7) - 87.7) < 1e-9 and G.seed_total(3.2, None) == 3.2
    assert abs(G.seed_total(12.49, 12.5) - 12.49) < 1e-9          # 0.1 rad 刻み丸め対策
    note(
        f'±12.5 rad を {int(250 * step / 25)} 回またいで往復し、'
        f'差は最大 {worst * 1000:.2f} mrad。起動時は内部の位置（0.1 rad 刻み）から周回を決める')


@case('帰還の読み：状態・返事の回数・内部の位置（不明は None）・位置・速度・トルク')
def test_decode():
    """帰還フレームの復号検証."""
    rx = {M.dev_steer: [0] * 24, M.dev_drive: [0] * 24}
    rx[M.dev_steer][6:12] = [1, 42, -32768, -3769, 150, -250]
    rx[M.dev_drive][18:24] = [15, 7, 123, 100, 4484, 900]
    dec = G.decode(M, rx)
    s, d = dec['steer'][1], dec['drive'][3]
    assert (s['status'], s['count'], s['internal']) == (1, 42, None), s
    assert abs(s['pos'] + 3.769) < 1e-9 and abs(s['vel'] - 1.5) < 1e-9
    assert abs(s['torque'] + 0.25) < 1e-9
    assert d['status'] == G.ST_LATCHED and abs(d['internal'] - 12.3) < 1e-9
    assert abs(d['vel'] - 44.84) < 1e-9
    deltas, speeds = G.module_state(M, dec, [r['pos'] for r in dec['steer']], ZEROS)
    assert abs(deltas[1] + 1.0) < 1e-3 and abs(speeds[3] - 1.25) < 1e-3, (deltas, speeds)
    assert G.decode(M, {M.dev_steer: rx[M.dev_steer]})['drive'] == [None] * 4
    try:
        G.decode(M, {M.dev_steer: [0] * 23})
    except ValueError:
        pass
    else:
        raise AssertionError('長さ 23 のフレームを通した')
    note('モータの位置 −3.769 rad → δ −57.3°、出力軸 44.84 rad/s → 車輪 1.25 m/s')


@case('原点：フォトセンサが切り替わった位置から、δ = 0 のモータの位置を出す')
def test_home():
    """zero_from_home の符号・ギア比整合性検証."""
    q = G.load(motors={
        'steer': [
            {'module': i, 'index': i, 'sign': -1 if i == 2 else 1, 'home': 0.5}
            for i in range(4)
        ],
        'drive': [{'module': i, 'index': i, 'sign': 1} for i in range(4)],
    })
    for mot in q.steer:
        zero = G.zero_from_home(q, mot, 4.0)
        tgt, flag = G.steer_target(q, mot, 0.5, 4.0, zero)
        assert flag is None and abs(tgt - 4.0) < 1e-9, (tgt, flag)
    note('home 0.5 rad の所で切り替わったなら、δ = 0.5 rad の目標はその位置になる（向き −1 でも同じ）')


@case('ありえない割り当て表は断る')
def test_refuse():
    """不正な YAML 設定の拒否検証."""
    def motors(changes=None, n=4):
        rows = [
            {'module': i, 'index': i, 'sign': 1, 'home': 0.0, **(changes or {}).get(i, {})}
            for i in range(n)
        ]
        return {'steer': rows, 'drive': [{'module': i, 'index': i, 'sign': 1} for i in range(4)]}

    cases = {
        '知らないキー': {'rate_hzz': 50},
        'index の重なり': {'motors': motors({1: {'index': 0}})},
        'sign が 0': {'motors': motors({2: {'sign': 0}})},
        'index が 4': {'motors': motors({3: {'index': 4}})},
        'module の抜け': {'motors': motors({3: {'module': 5}})},
        'ステアと駆動の台数違い': {'motors': motors(n=3)},
        '周期 20 Hz': {'rate_hz': 20.0},
        '途絶の時間が 0': {'cmd_timeout': 0.0},
        'デバイス ID の重なり': {'devices': {'steer': 201, 'drive': 201, 'inputs': 101}},
        '速さの上限がスロットの外': yaml_with(**{'drive.vel_max': 400.0}),
        '位置の範囲がスロットの外': yaml_with(**{'steer.pos_limit': 40.0}),
        '位置の範囲が ±90° に届かない': yaml_with(**{'steer.pos_limit': 5.0}),
        '負のギア比': yaml_with(**{'steer.gear': -3.77}),
    }
    for name, override in cases.items():
        refuses(**override)
    note(f'{len(cases)} 通り：' + '・'.join(cases))


@case('どれを送るか：起動直後は脱力、合図で走る、フリーズで保つ、非常停止・途絶・故障で脱力して次の合図まで戻らない')
def test_gate():
    """Gate 状態遷移の検証."""
    ok = {'cmd_age': 0.01, 'fb_age': 0.01, 'statuses': [1] * 8}
    g = G.Gate(M)
    assert g.step(**ok) == ('relax', 'startup')
    assert g.step(arm=True, **ok) == ('run', None)
    assert g.step(freeze=True, **ok) == ('hold', 'freeze')
    assert g.step(**ok) == ('run', None)                                 # フリーズ解除で再開
    assert g.step(freeze=True, estop=True, **ok) == ('relax', 'estop')   # 非常停止の方が強い
    assert g.step(**ok) == ('relax', 'estop')                            # 離しても戻らない
    assert g.step(arm=True, estop=True, **ok) == ('relax', 'estop')      # 押下中は合図を断る
    assert g.step(arm=True, **ok) == ('run', None)
    bad_cases = (
        ({**ok, 'cmd_age': 0.25}, 'cmd_lost'),
        ({**ok, 'fb_age': 0.25}, 'fb_lost'),
        ({**ok, 'cmd_age': None}, 'cmd_lost'),
        ({**ok, 'fb_age': None}, 'fb_lost'),
        ({**ok, 'statuses': [1, 1, 15, 1]}, 'latched'),
        ({**ok, 'statuses': [1, 13]}, 'fault:13'),
    )
    for bad, why in bad_cases:
        assert g.step(**bad) == ('relax', why), (bad, why)
        assert g.step(**ok) == ('relax', why)
        assert g.step(arm=True, **bad) == ('relax', why)
        assert g.step(arm=True, **ok) == ('run', None)
    assert g.step(freeze=True, **{**ok, 'cmd_age': 0.25}) == ('relax', 'cmd_lost')
    note('途絶は 0.2 s。フリーズ中に上流が途切れても脱力する（保つのは、上流が生きているときだけ）')


@case('スタートスイッチ：口の読み方と、押したを1回ずつ取り出す（離した後だけ・押しっぱなしで起動したら数えない・途絶は数えない）')
def test_start_switch():
    """スタートスイッチ StartSwitch のエッジ抽出・チャタリング除去の検証."""
    m = G.load(**yaml_with(**{'inputs.start': [2, 3], 'inputs.team': None}))
    assert G.switch_slot(m, 'start') == 7 and G.switch_slot(m, 'team') is None
    frame = [0] * G.N_SLOTS
    assert G.switch_pressed(m, frame, 'start') is False
    frame[7] = 1
    assert G.switch_pressed(m, frame, 'start') is True
    assert G.switch_pressed(m, frame, 'team') is None
    assert G.switch_pressed(m, [1] * 5, 'start') is None
    assert 'inputs.start' in refuses(**yaml_with(**{'inputs.start': [5, 1]}))
    assert 'inputs.start' in refuses(**yaml_with(**{'inputs.start': [1, 4]}))
    assert 'inputs.start' in refuses(**yaml_with(**{'inputs.start': 3}))
    assert '同じ口' in refuses(**yaml_with(**{'inputs.start': [1, 1], 'inputs.team': [1, 1]}))

    def run(reads, dt=0.01):
        sw, fired = G.StartSwitch(), []
        for i, r in enumerate(reads):
            if sw.step(i * dt, r):
                fired.append(i)
        return fired

    off, on = [False] * 10, [True] * 10
    assert len(run(off + on)) == 1, '離した後に押して1回にならない'
    assert run(off + on)[0] >= 10 + 5, '押してから 0.05 s 待たずに数えた'
    assert len(run(off + on * 5)) == 1, '押している間に繰り返した'
    assert len(run(off + on + off + on)) == 2, '2回押して2回にならない'
    seq = run(on * 3 + off + on)
    assert seq == [seq[0]] and seq[0] >= 40, '押したまま起動したのに、離す前に数えた'
    assert run(off + [True, True, False] * 5 + off) == [], 'チャタリングを数えた'
    assert run(off + [None] * 10 + on[:3] + [None] * 10) == [], '途絶を挟んだ短い読みを数えた'
    note('ノード 2 の SW3 → スロット 7。押す・離すはそれぞれ 0.05 s 続いてから決める')


@case('陣営スイッチ・非常停止のボタン・フリーズの受信機：読みの向き・ばたつきを除く・途絶で状態を変えない・ありえない読み方は断る')
def test_inputs():
    """機体入力読み取りと Level デバウンサの検証."""
    m = G.load()
    assert G.switches(m) == ['start', 'team', 'estop', 'freeze'], G.switches(m)
    sl = {n: G.switch_slot(m, n) for n in G.switches(m)}
    frame = [0] * G.N_SLOTS
    assert G.team_of(m, frame) == 'red'
    assert G.estop_held(m, frame) is True and G.freeze_wanted(m, frame) is False
    frame[sl['team']], frame[sl['estop']], frame[sl['freeze']] = 1, 1, 1
    assert G.team_of(m, frame) == 'blue'
    assert G.estop_held(m, frame) is False and G.freeze_wanted(m, frame) is True
    assert G.team_of(m, None) is None and G.estop_held(m, [0] * 5) is None
    assert G.freeze_wanted(m, None) is None

    q = G.load(**yaml_with(**{
        'input_rules.team_on': 'red', 'input_rules.estop_on': 1, 'input_rules.freeze_on': 0,
    }))
    assert G.team_of(q, frame) == 'red'
    assert G.estop_held(q, frame) is True and G.freeze_wanted(q, frame) is False

    u = G.load(**yaml_with(**{'inputs.estop': None}))
    assert 'estop' not in G.switches(u) and G.estop_held(u, frame) is None
    bad_rules = (
        {'input_rules.team_on': 'green'}, {'input_rules.estop_on': 2},
        {'input_rules.freeze_on': True}, {'input_rules.lost': 'stop'},
        {'input_rules.debounce': 0.0},
    )
    for bad in bad_rules:
        assert 'input_rules' in refuses(**yaml_with(**bad)), bad
    assert '同じ口' in refuses(**yaml_with(**{'inputs.freeze': [1, 3]}))

    def run(reads, dt=0.01):
        lv, out = G.Level(), []
        for i, r in enumerate(reads):
            st, ch = lv.step(i * dt, r)
            if ch:
                out.append((i, st))
        return out, lv.state

    ch, st = run([False] * 10)
    assert ch == [(5, False)] and st is False, ch
    ch, st = run([False] * 10 + [True, False] * 5 + [False] * 3)
    assert ch == [(5, False)], f'0.05 s 続かない読みで変えた {ch}'
    ch, st = run([True] * 10 + [None] * 50)
    assert st is True and ch == [(5, True)], '途絶で状態を変えた'
    ch, st = run([None] * 10 + [True] * 3)
    assert st is None and ch == [], '0.05 s 続く前に決めた'
    note(
        f'口は {sl}。陣営 0 → red・1 → blue、非常停止は 0（押す・断線）で押している、'
        'フリーズは 1 で止めろ（どれも仮）。読みは 0.05 s 続いてから決め、途絶の間は状態を残す')


@case('足回りの値（swerve.yaml）と食い違わない')
def test_swerve_yaml():
    """逆運動学の swerve.yaml との整合性検証."""
    import swerve as S
    p = S.load()
    top = M.drive_vel_max / M.drive_gear * M.wheel_radius
    assert p.n == M.n, (p.n, M.n)
    assert p.wheel_speed_max <= top + 1e-9, (p.wheel_speed_max, top)
    note(
        f'モジュール {p.n} つ。シムの車輪の上限 {p.wheel_speed_max:g} m/s ≦ '
        f'実機の換算 {top:.3f} m/s')


def main():
    """全単体試験を実行して結果を表示する."""
    bad = 0
    print(f'値：{os.path.realpath(G.DEFAULT_PATH)}')
    for name, fn in TESTS:
        _NOTE.clear()
        try:
            fn()
            print(f"  OK   {name}：{'／'.join(_NOTE)}")
        except Exception as e:                                     # noqa: BLE001
            bad += 1
            print(f'  NG   {name}：{type(e).__name__} {e}')
    print()
    print(f"{'合格' if not bad else '不合格'} {len(TESTS)} 項目 / 指摘 {bad} 件")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
