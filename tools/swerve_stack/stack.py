"""制御スタック全体: MPPI (25 Hz) -> 速度整形 -> 運動学 -> 各軸 FF+PID+DOB (1 kHz).

    MPPI ----- u (機体速度指令) ----> 速度レートリミッタ -> nu_ref
                                                            |
                                               SwerveKinematics
                                         (ステア目標角, 駆動速度) x 4
                                              |                 |
                               SteerAxisController x4   DriveAxisController x4
                              (参照軌道+FF+PID+DOB)     (FF+PI+DOB, 坂の重力FF)
"""

import math
from dataclasses import dataclass

import numpy as np

from drive_axis import DriveAxisConfig, DriveAxisController
from kinematics import SwerveKinematics
from mppi import SwerveMPPI
from plant import GRAVITY_MPS2
from steer_axis import SteerAxisConfig, SteerAxisController


@dataclass
class NominalModel:
    """コントローラが「そうだと思っている」値。真のプラントとは意図的にずらす."""
    mass_kg: float = 20.0
    wheel_radius_m: float = 0.05
    wheel_inertia_kgm2: float = 0.003
    wheel_damping_nms: float = 0.005
    steer_inertia_kgm2: float = 0.002
    steer_damping_nms: float = 0.005


class SwerveController:
    def __init__(self, positions, steer_limit_rad, ref_fn, nominal: NominalModel,
                 use_ff: bool, use_dob: bool, steer_aware_mppi: bool = True,
                 mppi_period_s: float = 0.04, mppi_seed: int = 0):
        self.nom = nominal
        self.positions = positions
        self.mppi = SwerveMPPI(positions, steer_limit_rad, ref_fn, steer_aware=steer_aware_mppi,
                               seed=mppi_seed)
        self.mppi_period_s = mppi_period_s
        self._next_mppi_t = 0.0
        self.arrive_tol_m = 0.02
        self.arrive_tol_rad = math.radians(2.0)
        self.u_cmd = np.zeros(3)
        self.nu_ref = np.zeros(3)
        self.acc_max = np.array([3.0, 3.0, 6.0])
        self.kin = SwerveKinematics(positions, steer_limit_rad)

        wn = 30.0
        js = nominal.steer_inertia_kgm2
        self.steer = [
            SteerAxisController(SteerAxisConfig(
                j_nom_kgm2=js, b_nom_nms=nominal.steer_damping_nms,
                torque_max_nm=1.5, kp=js * wn * wn, ki=4.0, kd=2.0 * js * wn,
                integral_limit_nm=0.5, vel_max_rad_s=8.0, acc_max_rad_s2=80.0,
                use_ff=use_ff, use_dob=use_dob, dob_cutoff_rad_s=200.0, dob_limit_nm=0.6))
            for _ in range(4)
        ]
        j_eq = nominal.wheel_inertia_kgm2 + nominal.mass_kg * nominal.wheel_radius_m ** 2 / 4.0
        wc = 40.0
        self.drive = [
            DriveAxisController(DriveAxisConfig(
                j_eq_nom_kgm2=j_eq, b_nom_nms=nominal.wheel_damping_nms,
                kp=j_eq * wc, ki=j_eq * wc * wc / 5.0,
                use_ff=use_ff, use_dob=use_dob))
            for _ in range(4)
        ]
        self.log_d_hat_drive = np.zeros(4)

    def update(self, t_s, dt_s, pos, psi, twist_b, delta, ddelta, wheel_speed, slope_meas_rad):
        # 1. 上位計画 (MPPI) は低い周期で
        if t_s >= self._next_mppi_t - 1e-9:
            self.u_cmd = self.mppi.solve(t_s, pos, psi, twist_b, delta)
            self._next_mppi_t += self.mppi_period_s
            # 到着判定: 目標が止まっていて誤差が許容内なら、補正をやめて停止を保つ。
            # これが無いと、MPPI の小さな補正指令でステアが振り回され続ける。
            rx, ry, rpsi, *rv = self.mppi.ref_fn(t_s)
            ref_still = np.max(np.abs(rv)) < 1e-3
            pos_ok = math.hypot(rx - pos[0], ry - pos[1]) < self.arrive_tol_m
            psi_ok = abs(math.atan2(math.sin(psi - rpsi), math.cos(psi - rpsi))) < self.arrive_tol_rad
            if ref_still and pos_ok and psi_ok:
                self.u_cmd = np.zeros(3)
                if self.mppi.U is not None:
                    self.mppi.U[:] = 0.0

        # 2. 速度指令を加速度制限つきで整形（FF に使う加速度もここで決まる）
        # 軸ごとに clip すると速度ベクトルの向きが歪む（vx が加速中に vy だけ先に到達する）。
        # 独ステでは向き = ステア角なので、差分ベクトル全体を同じ倍率で縮める。
        step = self.acc_max * dt_s
        diff = self.u_cmd - self.nu_ref
        ratio = np.max(np.abs(diff) / step)
        self.nu_ref = self.nu_ref + (diff / ratio if ratio > 1.0 else diff)

        # 3. 逆運動学 + 角度解決
        steer_target, drive_mps = self.kin.command(self.nu_ref, delta)

        # 4. 各軸
        tau_s = np.array([
            ctrl.update(steer_target[i], delta[i], ddelta[i], dt_s)
            for i, ctrl in enumerate(self.steer)
        ])
        r_w = self.nom.wheel_radius_m
        slope_force = self.nom.mass_kg * GRAVITY_MPS2 * math.sin(slope_meas_rad) / 4.0
        tau_known = r_w * slope_force * np.cos(psi + delta)   # 坂を登る向きの成分だけ車輪が受け持つ
        tau_w = np.array([
            ctrl.update(drive_mps[i] / r_w, wheel_speed[i], tau_known[i], dt_s)
            for i, ctrl in enumerate(self.drive)
        ])
        self.log_d_hat_drive = np.array([c.d_hat for c in self.drive])
        return tau_s, tau_w
