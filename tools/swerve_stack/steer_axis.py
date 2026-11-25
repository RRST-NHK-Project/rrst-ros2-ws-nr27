"""ステア1軸の位置制御: 参照軌道 + FF + PID + DOB をまとめる.

        theta_target
             |
     [ TrapezoidalReference ] --> theta_ref, omega_ref, alpha_ref
             |                          |
             |                   [ MotorFeedforward ] --> torque_ff
             v
   error = theta_ref - theta_meas
   error_rate = omega_ref - omega_meas
             |
          [ PID ] --> torque_pid
             |
   torque_cmd = torque_ff + torque_pid - d_hat
                                          ^
                     [ DisturbanceObserver ] <-- omega_meas, torque_applied

役割分担:
  FF  : 目標から計算できる既知の成分を先に出す
  DOB : 計算できない成分（モデル誤差・外乱）を後から打ち消す
  PID : それでも残った分を埋める
"""

from dataclasses import dataclass

from dob import DisturbanceObserver
from feedforward import MotorFeedforward
from pid import PID
from ref_profile import TrapezoidalReference


@dataclass
class SteerAxisConfig:
    # 公称モデル (コントローラが「そうだと思っている」値)
    j_nom_kgm2: float
    b_nom_nms: float
    coulomb_nom_nm: float = 0.0
    torque_max_nm: float = 1.0
    # 位置PID
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit_nm: float = 0.3
    # 参照軌道
    vel_max_rad_s: float = 10.0
    acc_max_rad_s2: float = 100.0
    # 各機能のオン/オフ（比較実験用）
    use_ff: bool = True
    use_dob: bool = True
    dob_cutoff_rad_s: float = 200.0
    dob_limit_nm: float = 0.3


class SteerAxisController:
    def __init__(self, cfg: SteerAxisConfig):
        self.cfg = cfg
        self.ref = TrapezoidalReference(cfg.vel_max_rad_s, cfg.acc_max_rad_s2)
        self.ff = MotorFeedforward(cfg.j_nom_kgm2, cfg.b_nom_nms, cfg.coulomb_nom_nm)
        self.pid = PID(cfg.kp, cfg.ki, cfg.kd, cfg.integral_limit_nm)
        self.dob = DisturbanceObserver(
            cfg.j_nom_kgm2, cfg.b_nom_nms, cfg.dob_cutoff_rad_s, cfg.dob_limit_nm
        )
        self._last_cmd_nm = 0.0
        self._last_applied_nm = 0.0
        self.debug = {}

    def reset(self, theta_rad: float) -> None:
        self.ref.reset(theta_rad)
        self.pid.reset()
        self.dob.reset()
        self._last_cmd_nm = 0.0
        self._last_applied_nm = 0.0

    def update(
        self,
        theta_target_rad: float,
        theta_meas_rad: float,
        omega_meas_rad_s: float,
        dt_s: float,
    ) -> float:
        cfg = self.cfg
        theta_ref, omega_ref, alpha_ref = self.ref.update(theta_target_rad, dt_s)

        torque_ff = self.ff.torque_nm(omega_ref, alpha_ref) if cfg.use_ff else 0.0

        error = theta_ref - theta_meas_rad
        error_rate = omega_ref - omega_meas_rad_s

        # 前の周期で飽和していて、かつ偏差が飽和をさらに深める向きなら積分を止める
        freeze = (self._last_cmd_nm != self._last_applied_nm) and (
            error * self._last_cmd_nm > 0.0
        )
        torque_pid = self.pid.update(error, error_rate, dt_s, freeze_integral=freeze)

        # DOB には「前の周期に実際に出した（飽和後の）トルク」を渡す。
        # 今の周期の指令はまだ決まっていないので、1周期の遅れは設計上避けられない。
        d_hat = (
            self.dob.update(omega_meas_rad_s, self._last_applied_nm, dt_s)
            if cfg.use_dob
            else 0.0
        )

        torque_cmd = torque_ff + torque_pid - d_hat
        torque_applied = max(-cfg.torque_max_nm, min(cfg.torque_max_nm, torque_cmd))

        self._last_cmd_nm = torque_cmd
        self._last_applied_nm = torque_applied
        self.debug = {
            "theta_ref": theta_ref,
            "omega_ref": omega_ref,
            "alpha_ref": alpha_ref,
            "torque_ff": torque_ff,
            "torque_pid": torque_pid,
            "d_hat": d_hat,
            "torque_cmd": torque_cmd,
            "torque_applied": torque_applied,
        }
        return torque_applied
