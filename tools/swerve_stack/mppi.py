"""MPPI: 機体速度指令 (vx, vy, omega) [機体座標] を決める上位の計画器.

予測モデル（dt = 0.04 s = 呼び出し周期, ホライズン T=25 ステップ = 1 s）
  状態  : 位置 p, 姿勢 psi, 機体速度 nu=(vx,vy,omega), ステア角 delta_i
  入力  : 機体速度指令 u
  1. ステア : 目標角 = resolve(atan2(モジュール速度(u)), delta)
              delta は角速度上限 delta_dot_max で目標へ近づく
  2. 速度   : nu' = nu + clip((c*u - nu)/tau_v, +-a_max) dt
              c = mean_i max(0, cos(目標角 - delta_i))   向きが合っていないと進めない
  3. 姿勢   : p' = p + R(psi) nu_xy dt,  psi' = psi + omega dt

コスト（各ステップの和 + 終端）
  q_pos |p - p_ref|^2 + q_psi wrap(psi - psi_ref)^2 + q_vel |u - u_ref|^2
  + r_du |u_k - u_{k-1}|^2
  + w_travel sum|d delta| + w_lim sum max(0, |delta| - (L - margin))^2
  + w_align sum (1 - cos(目標角 - delta))

更新（情報理論的 MPPI の簡略形）
  U_k = U + eps_k,  eps_k ~ N(0, Sigma)
  w_k = exp(-(S_k - min S) / lambda) / Z
  U  <- U + sum_k w_k eps_k

steer_aware=False にすると、ステア角を予測に入れない（全方向移動ロボットとして扱う）。
有限旋回でステアを予測に入れる意味を比べるためのスイッチ。
"""

import math

import numpy as np

from kinematics import module_velocities, resolve_angles


class SwerveMPPI:
    def __init__(
        self,
        positions: np.ndarray,
        steer_limit_rad: float,
        ref_fn,
        steer_aware: bool = True,
        samples: int = 300,
        horizon: int = 25,
        dt_s: float = 0.04,
        seed: int = 0,
    ):
        self.positions = positions
        self.limit = steer_limit_rad
        self.ref_fn = ref_fn            # t -> (x, y, psi, vx_b, vy_b, omega)
        self.steer_aware = steer_aware
        self.K = samples
        self.T = horizon
        self.dt = dt_s
        self.rng = np.random.default_rng(seed)

        self.sigma = np.array([0.15, 0.15, 0.15])
        self.u_max = np.array([2.0, 2.0, 3.0])
        self.a_max = np.array([3.0, 3.0, 6.0])
        self.tau_v = 0.04
        self.steer_rate_max = 8.0
        self.lam = 20.0
        self.noise_smooth = 0.8
        self.ess = float(samples)

        self.q_pos, self.q_psi, self.q_vel = 1000.0, 40.0, 5.0
        self.r_du = 20.0
        self.w_travel, self.w_lim, self.w_align = 5.0, 300.0, 1.0
        self.lim_margin = 0.25
        self.terminal_gain = 5.0

        self.U = None
        self.last_u = np.zeros(3)

    def _ref_horizon(self, t0: float) -> np.ndarray:
        return np.array([self.ref_fn(t0 + (k + 1) * self.dt) for k in range(self.T)])

    def solve(self, t_s: float, pos, psi, twist_b, delta) -> np.ndarray:
        refs = self._ref_horizon(t_s)                       # (T, 6)
        if self.U is None:
            self.U = refs[:, 3:6].copy()
        else:
            self.U = np.vstack([self.U[1:], self.U[-1:]])  # ウォームスタート

        # 時間方向に相関したノイズ（1次ローパス）。白色ノイズだと指令がギザギザになる
        white = self.rng.normal(size=(self.K, self.T, 3))
        eps = np.empty_like(white)
        eps[:, 0] = white[:, 0]
        a = self.noise_smooth
        for k in range(1, self.T):
            eps[:, k] = a * eps[:, k - 1] + math.sqrt(1.0 - a * a) * white[:, k]
        eps *= self.sigma
        U_s = np.clip(self.U[None] + eps, -self.u_max, self.u_max)
        eps = U_s - self.U[None]

        K = self.K
        px = np.full(K, pos[0]); py = np.full(K, pos[1]); ps = np.full(K, psi)
        nu = np.tile(twist_b, (K, 1))
        dl = np.tile(delta, (K, 1))
        u_prev = np.tile(self.last_u, (K, 1))
        cost = np.zeros(K)
        step_rate = self.steer_rate_max * self.dt
        lim_in = self.limit - self.lim_margin

        for k in range(self.T):
            u = U_s[:, k, :]
            if self.steer_aware:
                mv = module_velocities(u, self.positions)            # (K,4,2)
                desired = np.arctan2(mv[..., 1], mv[..., 0])
                speed = np.hypot(mv[..., 0], mv[..., 1])
                target, _ = resolve_angles(desired, dl, self.limit)
                moving = speed > 0.10                          # 実機側の不感帯と合わせる
                target = np.where(moving, target, dl)
                move = np.clip(target - dl, -step_rate, step_rate)
                dl = dl + move
                align = np.cos(target - dl)
                # 不感帯以下のモジュールは駆動しない -> 小さすぎる補正指令は効かない、と予測に教える
                c = np.mean(moving * np.maximum(0.0, align), axis=1, keepdims=True)
                cost += self.w_travel * np.sum(np.abs(move), axis=1)
                cost += self.w_lim * np.sum(np.maximum(0.0, np.abs(dl) - lim_in) ** 2, axis=1)
                cost += self.w_align * np.sum(1.0 - align, axis=1)
            else:
                c = 1.0
            acc = (c * u - nu) / self.tau_v
            ratio = np.max(np.abs(acc) / self.a_max, axis=1, keepdims=True)
            acc = acc / np.maximum(ratio, 1.0)          # 向きを保ったまま加速度を制限（実機側と同じ）
            nu = nu + acc * self.dt
            cs, sn = np.cos(ps), np.sin(ps)
            px = px + (cs * nu[:, 0] - sn * nu[:, 1]) * self.dt
            py = py + (sn * nu[:, 0] + cs * nu[:, 1]) * self.dt
            ps = ps + nu[:, 2] * self.dt

            r = refs[k]
            gain = self.terminal_gain if k == self.T - 1 else 1.0
            dpsi = np.arctan2(np.sin(ps - r[2]), np.cos(ps - r[2]))
            cost += gain * (self.q_pos * ((px - r[0]) ** 2 + (py - r[1]) ** 2) + self.q_psi * dpsi ** 2)
            cost += self.q_vel * np.sum((u - r[3:6]) ** 2, axis=1)
            cost += self.r_du * np.sum((u - u_prev) ** 2, axis=1)
            u_prev = u

        w = np.exp(-(cost - cost.min()) / self.lam)
        w /= w.sum()
        self.ess = 1.0 / np.sum(w ** 2)    # 有効サンプル数. 1 に近いと「1本のくじ引き」になり指令が暴れる
        self.U = self.U + np.einsum("k,ktj->tj", w, eps)
        self.last_u = self.U[0].copy()
        return self.last_u
