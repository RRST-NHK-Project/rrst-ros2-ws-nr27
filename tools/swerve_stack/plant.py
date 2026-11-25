"""真のプラント: 4輪・有限旋回の独立ステア機体（平面3自由度＋各輪2軸）.

状態
  機体   : 位置 p=(x,y) [world], 姿勢 psi, 速度 v=(vx,vy) [world], 角速度 omega
  ステア : 角度 delta_i, 角速度 ddelta_i    (機体座標で測る, i=0..3)
  駆動輪 : 回転速度 Omega_i [rad/s]          (車輪軸側に換算済み)

入力
  ステアトルク tau_s_i, 駆動トルク tau_w_i  (どちらも軸側換算 [Nm])

タイヤ（粘着域を線形、摩擦円で飽和）
  接地点速度          v_c,i = v + omega * z x (R(psi) r_i)
  前後・横の成分      u_i = v_c,i . e_i,   w_i = v_c,i . n_i
  前後力              F_x,i = C_x (r_w Omega_i - u_i)
  横力                F_y,i = -C_y w_i
  摩擦円              |F_i| <= mu N_i   で比例縮小

運動方程式
  m dv/dt      = sum F_i + F_grav            F_grav = (-m g sin(theta), 0)  坂は world x 方向
  Iz domega/dt = sum (R r_i) x F_i
  J_w dOmega/dt = tau_w - r_w F_x - b_w Omega
  J_s dddelta/dt = tau_s - b_s ddelta - tau_c tanh(ddelta/eps) - trail F_y - (ストッパ)

数値積分は半陰的オイラー（速度を先に更新し、その速度で位置を進める）。
"""

import math
from dataclasses import dataclass, field

import numpy as np

GRAVITY_MPS2 = 9.81


def default_module_positions(half_size_m: float = 0.25) -> np.ndarray:
    h = half_size_m
    return np.array([[+h, +h], [+h, -h], [-h, +h], [-h, -h]], dtype=float)


@dataclass
class PlantParams:
    mass_kg: float = 25.0               # 機体＋積載
    inertia_z_kgm2: float = 1.2
    wheel_radius_m: float = 0.05
    wheel_inertia_kgm2: float = 0.004   # 車輪軸換算（モータ慣性 x 減速比^2 を含む）
    wheel_damping_nms: float = 0.006
    steer_inertia_kgm2: float = 0.003
    steer_damping_nms: float = 0.010
    steer_coulomb_nm: float = 0.05
    steer_limit_rad: float = math.radians(170.0)
    steer_stop_stiffness_nm_rad: float = 200.0
    trail_m: float = 0.005              # 横力がステアを戻そうとするモーメント腕
    tire_cx_ns_m: float = 2000.0
    tire_cy_ns_m: float = 2000.0
    friction_mu: float = 0.9
    module_positions_m: np.ndarray = field(default_factory=default_module_positions)


class SwervePlant:
    def __init__(self, params: PlantParams, slope_fn=None):
        self.p = params
        self.slope_fn = slope_fn if slope_fn is not None else (lambda x: 0.0)
        self.pos = np.zeros(2)
        self.psi = 0.0
        self.vel = np.zeros(2)
        self.omega = 0.0
        self.delta = np.zeros(4)
        self.ddelta = np.zeros(4)
        self.wheel_speed = np.zeros(4)
        self.tire_force_x = np.zeros(4)   # ログ用
        self.tire_force_y = np.zeros(4)

    def slope_rad(self) -> float:
        return self.slope_fn(self.pos[0])

    def body_twist(self):
        """機体座標系での (vx, vy, omega)."""
        c, s = math.cos(self.psi), math.sin(self.psi)
        vx_b = c * self.vel[0] + s * self.vel[1]
        vy_b = -s * self.vel[0] + c * self.vel[1]
        return np.array([vx_b, vy_b, self.omega])

    def step(self, tau_steer: np.ndarray, tau_wheel: np.ndarray, dt_s: float) -> None:
        p = self.p
        theta = self.slope_rad()
        normal_n = p.mass_kg * GRAVITY_MPS2 * math.cos(theta) / 4.0

        c, s = math.cos(self.psi), math.sin(self.psi)
        r = p.module_positions_m
        r_world = np.stack([c * r[:, 0] - s * r[:, 1], s * r[:, 0] + c * r[:, 1]], axis=1)

        heading = self.psi + self.delta
        e = np.stack([np.cos(heading), np.sin(heading)], axis=1)     # 車輪の前方向
        n = np.stack([-e[:, 1], e[:, 0]], axis=1)                     # 車輪の左方向

        v_contact = self.vel[None, :] + self.omega * np.stack([-r_world[:, 1], r_world[:, 0]], axis=1)
        u = np.einsum("ij,ij->i", v_contact, e)
        w = np.einsum("ij,ij->i", v_contact, n)

        fx = p.tire_cx_ns_m * (p.wheel_radius_m * self.wheel_speed - u)
        fy = -p.tire_cy_ns_m * w
        f_mag = np.hypot(fx, fy)
        scale = np.minimum(1.0, p.friction_mu * normal_n / np.maximum(f_mag, 1e-9))
        fx *= scale
        fy *= scale
        self.tire_force_x, self.tire_force_y = fx, fy

        f_world = fx[:, None] * e + fy[:, None] * n
        f_grav = np.array([-p.mass_kg * GRAVITY_MPS2 * math.sin(theta), 0.0])
        acc = (f_world.sum(axis=0) + f_grav) / p.mass_kg
        moment = np.sum(r_world[:, 0] * f_world[:, 1] - r_world[:, 1] * f_world[:, 0])
        alpha = moment / p.inertia_z_kgm2

        wheel_acc = (tau_wheel - p.wheel_radius_m * fx - p.wheel_damping_nms * self.wheel_speed) / p.wheel_inertia_kgm2

        # ステア: 摩擦 + 横力による戻しモーメント + 可動域端のストッパ
        stop = np.where(
            self.delta > p.steer_limit_rad,
            -p.steer_stop_stiffness_nm_rad * (self.delta - p.steer_limit_rad),
            np.where(
                self.delta < -p.steer_limit_rad,
                -p.steer_stop_stiffness_nm_rad * (self.delta + p.steer_limit_rad),
                0.0,
            ),
        )
        steer_acc = (
            tau_steer
            - p.steer_damping_nms * self.ddelta
            - p.steer_coulomb_nm * np.tanh(self.ddelta / 0.05)
            - p.trail_m * fy
            + stop
        ) / p.steer_inertia_kgm2

        # 半陰的オイラー
        self.vel += acc * dt_s
        self.omega += alpha * dt_s
        self.wheel_speed += wheel_acc * dt_s
        self.ddelta += steer_acc * dt_s
        self.pos += self.vel * dt_s
        self.psi += self.omega * dt_s
        self.delta += self.ddelta * dt_s
