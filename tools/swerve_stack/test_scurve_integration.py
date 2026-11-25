"""S 字を組み込んだ所のテスト (ref_profile.py のジャーク制限、MPPI の予測モデル)."""

import math

import numpy as np
import pytest

from mppi import SwerveMPPI
from plant import default_module_positions
from ref_profile import TrapezoidalReference

DT = 0.001
VEL, ACC, JERK = 8.0, 80.0, 1600.0   # stack.py のステアの値。JERK は立ち上げ 0.05 s


def step_response(target, jerk, duration=1.5):
    ref = TrapezoidalReference(VEL, ACC, jerk_max_rad_s3=jerk)
    out = [(0.0, 0.0, 0.0)]
    for _ in range(int(round(duration / DT))):
        out.append(ref.update(target, DT))
    return np.array(out)


@pytest.mark.parametrize('target', [0.05, 0.6, 1.0, 3.0, -2.0, -0.7])
def test_steer_reference_with_jerk_limit(target):
    out = step_response(target, JERK)
    theta, omega, alpha = out[:, 0], out[:, 1], out[:, 2]
    assert np.all(np.abs(np.diff(alpha)) <= JERK * DT + 1e-9)
    assert np.all(np.abs(alpha) <= ACC + 1e-9)
    assert np.all(np.abs(omega) <= VEL + 1e-9)
    assert abs(theta[-1] - target) < 1e-3
    overshoot = (theta - target) * math.copysign(1.0, target)
    # 加速の途中で減速の曲線に当たる 0.6〜1 rad で行き過ぎていた (ref_profile.py の d_ramp)
    assert overshoot.max() < 1e-3


def test_steer_reference_without_jerk_limit_is_unchanged_shape():
    # OFF (既定) は台形のまま: 加速度が 1 周期で上限まで飛ぶ
    out = step_response(1.0, None, duration=0.1)
    assert out[1, 2] == pytest.approx(ACC)


@pytest.mark.parametrize('profile', ['trapezoid', 'scurve'])
def test_mppi_runs_with_both_profiles(profile):
    def ref(t):
        return (min(t, 1.0), 0.0, 0.0, 1.0 if t < 1.0 else 0.0, 0.0, 0.0)
    mppi = SwerveMPPI(default_module_positions(), math.radians(170.0), ref, samples=64, horizon=10,
                      profile=profile, accel_max=[3.0, 3.0, 6.0], jerk_norm_per_s=5.0)
    u = mppi.solve(0.0, np.zeros(2), 0.0, np.zeros(3), np.zeros(4), accel=np.array([0.5, 0.0, 0.0]))
    assert u.shape == (3,)
    assert np.all(np.isfinite(u))


def test_mppi_rejects_unknown_profile():
    with pytest.raises(ValueError):
        SwerveMPPI(default_module_positions(), 3.0, lambda t: (0,) * 6, profile='s')
