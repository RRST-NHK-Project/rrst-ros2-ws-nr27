"""逆運動学と、有限旋回の角度解決（numpy でベクトル化. MPPI の予測でも同じ関数を使う）.

  モジュール速度   v_i = (vx - omega*y_i,  vy + omega*x_i)          (機体座標)
  向きと速さ       phi_i = atan2(v_i),   s_i = |v_i|
  角度解決         候補 phi_i + k*pi (k が奇数なら駆動反転) のうち
                   可動域 [-L, L] に入り、現在角との差が最小のもの
"""

import math

import numpy as np

_K = np.arange(-3, 4)                    # 候補 phi + k*pi
_FLIPPED = (_K % 2) != 0


def module_velocities(twist: np.ndarray, positions: np.ndarray) -> np.ndarray:
    """twist: (..., 3) -> (..., 4, 2)."""
    vx = twist[..., 0:1]
    vy = twist[..., 1:2]
    om = twist[..., 2:3]
    mvx = vx - om * positions[:, 1]
    mvy = vy + om * positions[:, 0]
    return np.stack([mvx, mvy], axis=-1)


def resolve_angles(desired: np.ndarray, current: np.ndarray, limit_rad: float, flip_penalty: float = 0.05):
    """desired, current: 同じ形. 戻り値 (target, sign)."""
    cand = desired[..., None] + _K * math.pi
    inside = np.abs(cand) <= limit_rad
    cost = np.abs(cand - current[..., None]) + flip_penalty * _FLIPPED
    cost = np.where(inside, cost, np.inf)
    idx = np.argmin(cost, axis=-1)
    target = np.take_along_axis(cand, idx[..., None], axis=-1)[..., 0]
    sign = np.where(_FLIPPED[idx], -1.0, 1.0)
    # 可動域内に候補が無い（可動域 < 90度など）ときは端にクランプ
    none_ok = ~np.any(inside, axis=-1)
    target = np.where(none_ok, np.clip(desired, -limit_rad, limit_rad), target)
    sign = np.where(none_ok, 1.0, sign)
    return target, sign


class SwerveKinematics:
    """機体速度指令 -> 各モジュールの (ステア目標角, 駆動速度[m/s, 符号付き])."""

    def __init__(self, positions: np.ndarray, limit_rad: float,
                 start_mps: float = 0.10, stop_mps: float = 0.05):
        self.positions = positions
        self.limit_rad = limit_rad
        # 低速では向きが定義できない（ゼロベクトルに向きは無い）。
        # 小さな補正指令でステアが振り回されないよう、ヒステリシス付きの不感帯を置く。
        self.start_mps = start_mps
        self.stop_mps = stop_mps
        self.moving = np.zeros(len(positions), dtype=bool)
        self.last_target = np.zeros(len(positions))

    def command(self, twist: np.ndarray, steer_meas: np.ndarray):
        mv = module_velocities(twist, self.positions)
        speed = np.hypot(mv[:, 0], mv[:, 1])
        desired = np.arctan2(mv[:, 1], mv[:, 0])
        target, sign = resolve_angles(desired, steer_meas, self.limit_rad)

        self.moving = np.where(self.moving, speed > self.stop_mps, speed > self.start_mps)
        moving = self.moving
        target = np.where(moving, target, self.last_target)   # 停止中は向きを保持
        self.last_target = target

        # 角度が合うまで駆動を絞る（横滑り防止）
        scale = np.maximum(0.0, np.cos(target - steer_meas))
        drive = np.where(moving, sign * speed * scale, 0.0)
        return target, drive
