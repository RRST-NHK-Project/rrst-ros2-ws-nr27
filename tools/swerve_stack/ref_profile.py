"""目標角 → なめらかな参照軌道 (theta_ref, omega_ref, alpha_ref).

FF を効かせるには、目標の「速度」と「加速度」が必要になる。
ところが resolve() が返す目標角はステップ状に飛ぶので、そのままでは
速度も加速度も定義できない。そこで速度・加速度に上限を持つ参照軌道を挟む。

方式: 位置誤差から「出してよい最大速度」を決め、加速度制限つきで追従させる。
    v_des = sign(e) * min(vel_max, gain*|e|, sqrt(2*a_brake*|e|))
    alpha = clip((v_des - omega) / accel_tau, +-acc_max)
sqrt の項が「この距離で止まれる速度」で、近づくと自動で減速する。
加速度は不連続（ジャーク無制限）。

ジャーク制限 (jerk_max_rad_s3 を与えたとき。None なら上の台形のまま):
    v_des = sign(e) * min(vel_max, gain*|e|, stopping_speed(|e| - d_ramp, a_brake, J))
    (omega, alpha) = JerkLimitedFilter(acc_max, J, vel_max).update(v_des)
stopping_speed は sqrt(2*a_brake*|e|) の S 字版 (加速度 0 から J で減速して止まれる速度)。
d_ramp は、今の加速度 (目標へ向かう向きの分) を J で 0 まで下げる間に進む距離:
    T = a/J,  d_ramp = w T + a T^2/2 - J T^3/6
ジャーク制限があると、加速の途中で減速の曲線に当たっても、加速度を 0 に戻すまで速度が
a^2/(2J) 増え続ける。d_ramp だけ手前で曲線を見ないと、行き過ぎる
(2026-11-24、vel 8・acc 80・J 1600 で 1 rad のステップが 0.139 rad 行き過ぎた。入れた後は 0)。
alpha は S 字の加速度をそのまま返す (FF に使う。差分で計算し直さない)。
accel_tau は使わない (加速度はフィルタが J の範囲で決める)。

有限旋回: theta は可動域内の絶対角をそのまま扱う（巻き戻し不要）。
無限旋回: 誤差 e を wrap_pi した値で計算すれば、同じコードで最短回りになる。
"""

import math
from typing import Optional

from motion_profile import JerkLimitedFilter, stopping_speed


class TrapezoidalReference:
    def __init__(
        self,
        vel_max_rad_s: float,
        acc_max_rad_s2: float,
        position_gain_per_s: float = 10.0,
        brake_ratio: float = 0.8,
        accel_tau_s: float = 0.01,
        jerk_max_rad_s3: Optional[float] = None,
    ):
        self.vel_max_rad_s = vel_max_rad_s
        self.acc_max_rad_s2 = acc_max_rad_s2
        self.position_gain_per_s = position_gain_per_s
        self.brake_acc_rad_s2 = brake_ratio * acc_max_rad_s2
        self.accel_tau_s = accel_tau_s
        self.theta_rad = 0.0
        self.omega_rad_s = 0.0
        self.alpha_rad_s2 = 0.0
        self.jerk_max_rad_s3 = jerk_max_rad_s3
        self._jerk_filter = (
            JerkLimitedFilter(acc_max_rad_s2, jerk_max_rad_s3, vel_max_rad_s)
            if jerk_max_rad_s3 is not None else None)

    def reset(self, theta_rad: float) -> None:
        self.theta_rad = theta_rad
        self.omega_rad_s = 0.0
        self.alpha_rad_s2 = 0.0
        if self._jerk_filter is not None:
            self._jerk_filter.reset()

    def update(self, theta_target_rad: float, dt_s: float):
        if self._jerk_filter is not None:
            return self._update_jerk_limited(theta_target_rad, dt_s)
        error = theta_target_rad - self.theta_rad
        speed_cap = min(
            self.vel_max_rad_s,
            self.position_gain_per_s * abs(error),
            math.sqrt(2.0 * self.brake_acc_rad_s2 * abs(error)),
        )
        v_des = math.copysign(speed_cap, error)
        alpha = (v_des - self.omega_rad_s) / self.accel_tau_s
        self.alpha_rad_s2 = max(-self.acc_max_rad_s2, min(self.acc_max_rad_s2, alpha))
        self.omega_rad_s += self.alpha_rad_s2 * dt_s
        self.theta_rad += self.omega_rad_s * dt_s
        return self.theta_rad, self.omega_rad_s, self.alpha_rad_s2

    def _update_jerk_limited(self, theta_target_rad: float, dt_s: float):
        error = theta_target_rad - self.theta_rad
        sign = math.copysign(1.0, error)
        accel_forward = max(0.0, sign * self.alpha_rad_s2)   # 目標へ向かう向きの加速度
        ramp_s = accel_forward / self.jerk_max_rad_s3
        ramp_distance = max(0.0, sign * self.omega_rad_s * ramp_s + accel_forward * ramp_s ** 2 / 2.0
                            - self.jerk_max_rad_s3 * ramp_s ** 3 / 6.0)
        speed_cap = min(
            self.vel_max_rad_s,
            self.position_gain_per_s * abs(error),
            stopping_speed(max(0.0, abs(error) - ramp_distance), self.brake_acc_rad_s2,
                           self.jerk_max_rad_s3),
        )
        v_des = sign * speed_cap
        self.omega_rad_s, self.alpha_rad_s2 = self._jerk_filter.update(v_des, dt_s)
        self.theta_rad += self.omega_rad_s * dt_s
        return self.theta_rad, self.omega_rad_s, self.alpha_rad_s2
