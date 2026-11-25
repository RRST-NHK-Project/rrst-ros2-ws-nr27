"""駆動輪1軸の速度制御: FF + PI + DOB.

公称モデル（車輪軸から見た等価系）
    J_eq dOmega/dt + B_n Omega = tau - tau_known + d
    J_eq = J_w,n + (m_n / 4) r_w^2          車輪慣性 + 機体質量の1/4 を車輪軸に換算

  tau_known : 既知の負荷（坂の重力分. IMU のピッチから計算）
  d         : それ以外の全部（質量誤差, 他輪との干渉, 旋回時の連成, 未知の坂 ...）

制御則
    tau = J_eq alpha_ref + B_n Omega_ref + tau_known      (FF)
        + Kp e + Ki int e                                  (PI, e = Omega_ref - Omega)
        - d_hat                                            (DOB)

DOB への入力トルクは  tau_applied - tau_known  とする。
既知の負荷を FF で入れたのに DOB にそのまま渡すと、DOB は同じ負荷を外乱として
もう一度推定し、二重に補償してしまう。「FF で入れたもの = 公称モデルの一部」と
扱うのが整合の取り方。
"""

from dataclasses import dataclass

from dob import DisturbanceObserver
from pid import PID


@dataclass
class DriveAxisConfig:
    j_eq_nom_kgm2: float
    b_nom_nms: float
    torque_max_nm: float = 3.0
    kp: float = 0.6
    ki: float = 5.0
    integral_limit_nm: float = 2.0
    acc_max_rad_s2: float = 200.0     # 参照の角加速度制限
    use_ff: bool = True
    use_dob: bool = True
    dob_cutoff_rad_s: float = 150.0
    dob_limit_nm: float = 2.0


class DriveAxisController:
    def __init__(self, cfg: DriveAxisConfig):
        self.cfg = cfg
        self.pid = PID(cfg.kp, cfg.ki, 0.0, cfg.integral_limit_nm)
        self.dob = DisturbanceObserver(cfg.j_eq_nom_kgm2, cfg.b_nom_nms, cfg.dob_cutoff_rad_s, cfg.dob_limit_nm)
        self.omega_ref = 0.0
        self._last_cmd = 0.0
        self._last_applied = 0.0
        self._last_known = 0.0
        self.d_hat = 0.0

    def update(self, omega_target: float, omega_meas: float, tau_known: float, dt_s: float) -> float:
        cfg = self.cfg
        # 参照を加速度制限つきで動かし、その差分を alpha_ref とする
        step = cfg.acc_max_rad_s2 * dt_s
        new_ref = self.omega_ref + max(-step, min(step, omega_target - self.omega_ref))
        alpha_ref = (new_ref - self.omega_ref) / dt_s
        self.omega_ref = new_ref

        tau_ff = 0.0
        if cfg.use_ff:
            tau_ff = cfg.j_eq_nom_kgm2 * alpha_ref + cfg.b_nom_nms * self.omega_ref + tau_known
        known_for_dob = tau_known if cfg.use_ff else 0.0

        error = self.omega_ref - omega_meas
        freeze = (self._last_cmd != self._last_applied) and (error * self._last_cmd > 0.0)
        tau_pi = self.pid.update(error, 0.0, dt_s, freeze_integral=freeze)

        self.d_hat = 0.0
        if cfg.use_dob:
            self.d_hat = self.dob.update(omega_meas, self._last_applied - self._last_known, dt_s)

        cmd = tau_ff + tau_pi - self.d_hat
        applied = max(-cfg.torque_max_nm, min(cfg.torque_max_nm, cmd))
        self._last_cmd, self._last_applied, self._last_known = cmd, applied, known_for_dob
        return applied
