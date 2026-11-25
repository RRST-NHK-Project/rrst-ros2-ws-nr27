"""motion_profile.py のテスト (docs/scurve_prompt.md §5 の「式の検証」「逐次版」「ベクトル版」).

実行: cd tools/swerve_stack && python3 -m pytest -q test_motion_profile.py
"""

import math

import numpy as np
import pytest

from motion_profile import (JerkLimitedFilter, VectorJerkLimitedFilter, jerk_from_natural_frequency,
                            peak_velocity_rest_to_rest, plan_move, plan_rest_to_rest,
                            plan_velocity_change, stopping_speed, vector_step, velocity_change)

A = 3.0    # [m/s^2]
J = 15.0   # [m/s^3]
REL = 0.005  # §5 の許容誤差 0.5%


# ---------------------------------------------------------------------------
# 式の検証 (1 次元)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('v0, v1, case1, duration, distance', [
    (0.0, 2.0, True, 0.8667, 0.8667),
    (1.0, 2.0, True, 0.5333, 0.8000),
    (0.0, 0.4, False, 0.3266, 0.0653),
])
def test_velocity_change_table(v0, v1, case1, duration, distance):
    change = velocity_change(v0, v1, A, J)
    assert change.reaches_accel_max == case1
    assert change.duration == pytest.approx(duration, rel=REL)
    assert change.distance == pytest.approx(distance, rel=REL)


def test_deceleration_is_mirror_of_acceleration():
    up = velocity_change(0.0, 2.0, A, J)
    down = velocity_change(2.0, 0.0, A, J)
    assert down.duration == pytest.approx(up.duration)
    assert down.distance == pytest.approx(up.distance)


def test_rest_to_rest_reaching_vel_max():
    plan = plan_rest_to_rest(2.0, 2.0, A, J)
    assert plan.duration == pytest.approx(1.8667, rel=REL)
    # T = L/v_max + v_max/A + A/J
    assert plan.duration == pytest.approx(2.0 / 2.0 + 2.0 / A + A / J)


def test_peak_velocity_case1_without_vel_max():
    assert peak_velocity_rest_to_rest(2.0, A, J) == pytest.approx(2.168, rel=REL)


def test_peak_velocity_case2():
    # ケース 1 の式では 0.3245 (< A^2/J = 0.6) になるので、ケース 2 で計算し直す
    assert peak_velocity_rest_to_rest(0.1, A, J) == pytest.approx(0.3347, rel=REL)


def integrate_numerically(plan, step=1e-5):
    """計画のジャークだけを細かい刻みで積分し、終点の (位置, 速度, 加速度) を返す."""
    t = np.arange(0.0, plan.duration, step)
    jerk = np.array([plan.sample(ti + step / 2.0).jerk for ti in t])  # 区間の中点の値
    accel = plan.start.accel + np.concatenate([[0.0], np.cumsum(jerk * step)])
    velocity = plan.start.velocity + np.concatenate(
        [[0.0], np.cumsum((accel[:-1] + accel[1:]) / 2.0 * step)])
    position = plan.start.position + np.sum((velocity[:-1] + velocity[1:]) / 2.0 * step)
    rest = plan.duration - t[-1] - step  # 最後の刻みの端数 (負)。等速とみなして直す
    return position + velocity[-1] * rest, velocity[-1], accel[-1]


@pytest.mark.parametrize('plan', [
    plan_rest_to_rest(2.0, 2.0, A, J),          # 等速区間あり
    plan_rest_to_rest(2.0, math.inf, A, J),     # ケース 1 のピーク
    plan_rest_to_rest(0.1, 2.0, A, J),          # ケース 2 のピーク
    plan_rest_to_rest(-1.5, 2.0, A, J),         # 負の向き
    plan_velocity_change(1.0, 2.0, A, J),
    plan_move(1.0, 0.5, 0.2, 2.0, A, J),        # 初速・終速あり (二分法)
], ids=['cruise', 'case1', 'case2', 'negative', 'velocity_change', 'general'])
def test_numerical_integration_matches_plan_end(plan):
    position, velocity, accel = integrate_numerically(plan)
    end = plan.end()
    assert position == pytest.approx(end.position, abs=1e-4)
    assert velocity == pytest.approx(end.velocity, abs=1e-4)
    assert accel == pytest.approx(0.0, abs=1e-6)
    assert end.accel == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize('distance, v_max', [(2.0, 2.0), (2.0, math.inf), (0.1, 2.0), (-1.5, 2.0)])
def test_rest_to_rest_reaches_target_and_respects_limits(distance, v_max):
    plan = plan_rest_to_rest(distance, v_max, A, J)
    end = plan.end()
    assert end.position == pytest.approx(distance, abs=1e-9)
    assert end.velocity == pytest.approx(0.0, abs=1e-9)
    for t in np.linspace(0.0, plan.duration, 2001):
        s = plan.sample(t)
        assert abs(s.velocity) <= v_max + 1e-9
        assert abs(s.accel) <= A + 1e-9
        assert abs(s.jerk) <= J + 1e-9


def test_general_move_reaches_end_velocity():
    plan = plan_move(1.0, 0.5, 0.2, 2.0, A, J)
    end = plan.end()
    assert end.position == pytest.approx(1.0, abs=1e-9)
    assert end.velocity == pytest.approx(0.2, abs=1e-9)


def test_general_move_rejects_too_short_distance():
    with pytest.raises(ValueError):
        plan_move(0.01, 2.0, 0.0, 2.0, A, J)


@pytest.mark.parametrize('v0, v1', [(0.0, 2.0), (1.0, 2.0), (2.0, 0.5)])
def test_difference_from_trapezoid_is_a_over_j(v0, v1):
    # ケース 1 (dv >= A^2/J) のときだけ、ちょうど A/J になる。ケース 2 では A/J より小さい
    change = velocity_change(v0, v1, A, J)
    assert change.reaches_accel_max
    trapezoid_time = abs(v1 - v0) / A
    trapezoid_distance = (v0 + v1) / 2.0 * trapezoid_time
    assert change.duration - trapezoid_time == pytest.approx(A / J)
    assert change.distance - trapezoid_distance == pytest.approx((v0 + v1) / 2.0 * A / J)


def test_jerk_from_natural_frequency():
    # T_j = A/J が揺れの周期 1/f_n に一致する
    j = jerk_from_natural_frequency(A, 5.0)
    assert j == pytest.approx(15.0)
    assert A / j == pytest.approx(1.0 / 5.0)


# ---------------------------------------------------------------------------
# 逐次版 (1 次元)
# ---------------------------------------------------------------------------

DTS = [0.001, 0.01, 0.02, 0.04]  # マイコン 1 kHz、PC 100/50 Hz、MPPI 25 Hz


def run(targets, dt, vel_max=math.inf):
    f = JerkLimitedFilter(A, J, vel_max)
    out = [(0.0, 0.0)]
    for vt in targets:
        out.append(f.update(vt, dt))
    return np.array(out)


def check_limits(out, dt, vel_max=math.inf):
    """全周期で上限を守っているか.

    ジャークと加速度は離散でも厳密に守る (丸めの 1e-9 だけ許す)。
    速度は行き過ぎの分 (0.25 J dt^2、実測は約 0.11 J dt^2) まで許す。
    """
    v, a = out[:, 0], out[:, 1]
    assert np.all(np.abs(np.diff(a)) <= J * dt + 1e-9)
    assert np.all(np.abs(a) <= A + 1e-9)
    if math.isfinite(vel_max):
        assert np.all(np.abs(v) <= vel_max + 0.25 * J * dt * dt)


def reach_index(out, target):
    hit = np.nonzero((out[:, 0] == target) & (out[:, 1] == 0.0))[0]
    assert hit.size > 0, '目標に着いていない'
    return int(hit[0])


@pytest.mark.parametrize('dt', DTS)
def test_step_response_limits_and_overshoot(dt):
    n = int(round(2.0 / dt))
    out = run([2.0] * n, dt)
    check_limits(out, dt)
    assert out[:, 0].max() - 2.0 <= J * dt * dt


@pytest.mark.parametrize('dt', DTS)
def test_reach_time_matches_plan_within_one_period(dt):
    n = int(round(2.0 / dt))
    out = run([2.0] * n, dt)
    t_reach = reach_index(out, 2.0) * dt
    assert abs(t_reach - velocity_change(0.0, 2.0, A, J).duration) <= dt


@pytest.mark.parametrize('dt', DTS)
def test_no_oscillation_near_target(dt):
    n = int(round(2.0 / dt))
    out = run([2.0] * n, dt)
    a = out[:, 1]
    # 加速度が一度も逆向きに振れない
    assert not np.any(a[1:] * a[:-1] < 0.0)
    # 着いた後は動かない
    k = reach_index(out, 2.0)
    assert np.all(out[k:, 0] == 2.0)
    assert np.all(out[k:, 1] == 0.0)


@pytest.mark.parametrize('dt', DTS)
def test_target_reversal_during_acceleration(dt):
    # 加速中 (0.4 s、加速度が A に張り付いている) に -1 m/s へ変える
    n = int(round(3.0 / dt))
    k = int(round(0.4 / dt))
    out = run([2.0] * k + [-1.0] * (n - k), dt)
    check_limits(out, dt)
    assert out[-1, 0] == -1.0
    assert out[-1, 1] == 0.0
    assert -1.0 - out[:, 0].min() <= J * dt * dt


@pytest.mark.parametrize('dt', DTS)
def test_vel_max_clamps_target(dt):
    n = int(round(2.0 / dt))
    out = run([5.0] * n, dt, vel_max=1.5)
    check_limits(out, dt, vel_max=1.5)
    assert out[-1, 0] == 1.5


def test_stays_at_rest_with_zero_target():
    out = run([0.0] * 100, 0.01)
    assert np.all(out == 0.0)


def test_small_step_uses_case2_shape():
    # A に届かない小さな変化 (0 → 0.4、ケース 2) でも計画と 1 周期以内
    dt = 0.001
    out = run([0.4] * 1000, dt)
    check_limits(out, dt)
    assert out[:, 1].max() < A
    t_reach = reach_index(out, 0.4) * dt
    assert abs(t_reach - velocity_change(0.0, 0.4, A, J).duration) <= dt


# ---------------------------------------------------------------------------
# 止まれる速度 (ステアの参照軌道用)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('v', [0.05, 0.3, 0.6, 1.0, 2.0, 8.0])
def test_stopping_speed_inverts_velocity_change(v):
    # v から加速度 0 で 0 まで止まる距離を、逆に速度へ戻す (ケース 1・2 の両方)
    d = velocity_change(v, 0.0, A, J).distance
    assert stopping_speed(d, A, J) == pytest.approx(v, rel=1e-9)


# ---------------------------------------------------------------------------
# 揺れ (§3.3): T_j = 1/f_n で残留振動が消える
# ---------------------------------------------------------------------------

def residual_vibration(plan, natural_frequency_hz, step=1e-4):
    """減衰の無い 1 自由度の揺れ (x'' + w^2 x = -a) を計画の加速度で揺らし、終わった後の振幅を返す."""
    w = 2.0 * math.pi * natural_frequency_hz
    x = xd = 0.0
    for t in np.arange(0.0, plan.duration, step):
        a = plan.sample(t + step / 2.0).accel
        xd += (-a - w * w * x) * step
        x += xd * step
    return math.hypot(x, xd / w)


def test_residual_vibration_is_smallest_when_j_equals_a_times_fn():
    # L = 2 m・v_max = 2 m/s は、台形でも加速度の切り替わり (0, 2/3, 1, 5/3 s) の揺れが
    # 偶然打ち消し合って残留振動が 0 になる (等速の 1 s が周期 0.2 s の整数倍)。比べられないので 1.7 m にする
    f_n, length = 5.0, 1.7
    residual = {f: residual_vibration(plan_rest_to_rest(length, 2.0, A, A * f), f_n)
                for f in (3.5, 4.5, 5.0, 5.5, 7.0, 10.0)}
    trapezoid = residual_vibration(plan_rest_to_rest(length, 2.0, A, 1e6), f_n)
    assert min(residual, key=residual.get) == 5.0
    assert residual[5.0] < 1e-2 * trapezoid


# ---------------------------------------------------------------------------
# ベクトル版
# ---------------------------------------------------------------------------

ACC = np.array([3.0, 3.0, 6.0])  # stack.py の acc_max
JN = 5.0                         # J_n = J/A [1/s]


def run_vector(targets, dt):
    f = VectorJerkLimitedFilter(ACC, JN)
    vel, acc = [f.velocity], [f.accel]
    for target in targets:
        v, a = f.update(target, dt)
        vel.append(v)
        acc.append(a)
    return np.array(vel), np.array(acc)


def angle_from(v, direction):
    """v の向きと direction の向きの差 [rad] (v が 0 なら 0)."""
    n = np.linalg.norm(v)
    if n == 0.0:
        return 0.0
    u = direction / np.linalg.norm(direction)
    return float(np.arctan2(np.linalg.norm(np.cross(v / n, u)), np.dot(v / n, u)))


@pytest.mark.parametrize('target', [
    np.array([1.5, -0.8, 0.0]),   # 並進だけ
    np.array([0.0, 0.0, 2.5]),    # 回転だけ
    np.array([1.0, 1.2, -2.0]),   # 両方
], ids=['translation', 'rotation', 'both'])
@pytest.mark.parametrize('dt', [0.001, 0.04])
def test_vector_keeps_direction_and_jerk_limit(target, dt):
    vel, acc = run_vector([target] * int(round(2.0 / dt)), dt)
    # 向き: 加速の途中も目標の向きから変わらない
    assert max(angle_from(v, target) for v in vel) <= 1e-6
    # ジャーク: 正規化した空間で |da| <= J_n dt、|a| <= 1
    acc_n = acc / ACC
    assert np.all(np.linalg.norm(np.diff(acc_n, axis=0), axis=1) <= JN * dt + 1e-12)
    assert np.all(np.linalg.norm(acc_n, axis=1) <= 1.0 + 1e-12)
    # 着く
    assert np.allclose(vel[-1], target)
    assert np.allclose(acc[-1], 0.0)


@pytest.mark.parametrize('dt', [0.001, 0.01, 0.04])
def test_vector_with_one_axis_matches_1d_filter(dt):
    # 1 軸だけ動かすと、A = A_x、J = A_x J_n の 1 次元版と同じ値になる
    n = int(round(2.0 / dt))
    vel, acc = run_vector([np.array([2.0, 0.0, 0.0])] * n, dt)
    f = JerkLimitedFilter(ACC[0], ACC[0] * JN)
    ref = np.array([(0.0, 0.0)] + [f.update(2.0, dt) for _ in range(n)])
    assert np.allclose(vel[:, 0], ref[:, 0], atol=1e-12)
    assert np.allclose(acc[:, 0], ref[:, 1], atol=1e-12)
    assert np.all(vel[:, 1:] == 0.0)


@pytest.mark.parametrize('dt', [0.001, 0.04])
def test_vector_target_turns_and_reverses(dt):
    # 途中で向きを 90 度変え、その後に逆向きにする。J の範囲で回り込んで着く
    n = int(round(1.0 / dt))
    targets = ([np.array([2.0, 0.0, 0.0])] * n + [np.array([0.0, 2.0, 1.0])] * n
               + [np.array([-1.0, -1.0, 0.0])] * (2 * n))
    vel, acc = run_vector(targets, dt)
    acc_n = acc / ACC
    assert np.all(np.linalg.norm(np.diff(acc_n, axis=0), axis=1) <= JN * dt + 1e-12)
    assert np.all(np.linalg.norm(acc_n, axis=1) <= 1.0 + 1e-12)
    assert np.allclose(vel[-1], [-1.0, -1.0, 0.0])


def test_vector_step_batch_matches_single():
    # MPPI は (K, 3) でまとめて呼ぶ。1 本ずつ呼んだのと同じ値になる
    rng = np.random.default_rng(0)
    v = rng.normal(size=(50, 3))
    a = rng.normal(size=(50, 3)) * 0.3
    t = rng.normal(size=(50, 3))
    bv, ba = vector_step(v, a, t, JN, 0.04)
    for k in range(50):
        sv, sa = vector_step(v[k], a[k], t[k], JN, 0.04)
        assert np.array_equal(bv[k], sv) and np.array_equal(ba[k], sa)
