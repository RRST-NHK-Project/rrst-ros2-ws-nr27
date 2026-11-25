"""目標角 → なめらかな参照軌道 (theta_ref, omega_ref, alpha_ref).

FF を効かせるには、目標の「速度」と「加速度」が必要になる。
ところが resolve() が返す目標角はステップ状に飛ぶので、そのままでは
速度も加速度も定義できない。そこで速度・加速度に上限を持つ参照軌道を挟む。

方式: 位置誤差から「出してよい最大速度」を決め、加速度制限つきで追従させる。
    v_des = sign(e) * min(vel_max, gain*|e|, sqrt(2*a_brake*|e|))
    alpha = clip((v_des - omega) / accel_tau, +-acc_max)
sqrt の項が「この距離で止まれる速度」で、近づくと自動で減速する。
加速度は不連続（ジャーク無制限）。ジャークまで制限したいなら S字に拡張する。

有限旋回: theta は可動域内の絶対角をそのまま扱う（巻き戻し不要）。
無限旋回: 誤差 e を wrap_pi した値で計算すれば、同じコードで最短回りになる。
"""

import math


class TrapezoidalReference:
    def __init__(
        self,
        vel_max_rad_s: float,
        acc_max_rad_s2: float,
        position_gain_per_s: float = 10.0,
        brake_ratio: float = 0.8,
        accel_tau_s: float = 0.01,
    ):
        self.vel_max_rad_s = vel_max_rad_s
        self.acc_max_rad_s2 = acc_max_rad_s2
        self.position_gain_per_s = position_gain_per_s
        self.brake_acc_rad_s2 = brake_ratio * acc_max_rad_s2
        self.accel_tau_s = accel_tau_s
        self.theta_rad = 0.0
        self.omega_rad_s = 0.0
        self.alpha_rad_s2 = 0.0

    def reset(self, theta_rad: float) -> None:
        self.theta_rad = theta_rad
        self.omega_rad_s = 0.0
        self.alpha_rad_s2 = 0.0

    def update(self, theta_target_rad: float, dt_s: float):
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
