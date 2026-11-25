"""外乱オブザーバ（DOB）: 速度を測って、モデルで説明できない外乱トルクを推定する.

対象モデル (公称):   J_n * domega/dt + B_n * omega = torque + d
                      d は「公称モデルで説明できないトルク」の全部
                      (慣性誤差, 摩擦誤差, 負荷, 他軸からの干渉 ...)

推定式:   d_hat = Q(s) * ( J_n * s * omega + B_n * omega - torque )
          Q(s) = g / (s + g)         ローパス (遮断 g [rad/s])

s (微分) を直接書くと数値微分でノイズが増えるので、
    Q(s) * J_n * s = J_n * g * (1 - Q(s))
の恒等式を使い、LPF の出力との差だけで書き直してある。
この形なら明示的な微分が1つも出てこない。

使い方:
    d_hat = dob.update(omega_meas, torque_applied, dt)
    torque_cmd = torque_from_controller - d_hat     # 外乱を打ち消す向き

注意:
  * torque_applied は「実際にアクチュエータへ出した値」(飽和後) を渡すこと。
    飽和前の値を渡すと、飽和した分を外乱と誤認して巻き上がる。
  * g を上げるほど外乱抑圧は強くなるが、測定ノイズとモデル誤差に弱くなる。
    目安は 制御周期 dt に対して g * dt <= 0.2 程度まで。
"""

import math


class DisturbanceObserver:
    def __init__(
        self,
        j_nom_kgm2: float,
        b_nom_nms: float,
        cutoff_rad_s: float,
        limit_nm: float = float("inf"),
    ):
        self.j_nom_kgm2 = j_nom_kgm2
        self.b_nom_nms = b_nom_nms
        self.cutoff_rad_s = cutoff_rad_s
        self.limit_nm = limit_nm
        self._omega_lpf = 0.0
        self._torque_lpf = 0.0
        self.d_hat_nm = 0.0

    def reset(self, omega_rad_s: float = 0.0, torque_nm: float = 0.0) -> None:
        self._omega_lpf = omega_rad_s
        self._torque_lpf = torque_nm
        self.d_hat_nm = 0.0

    def update(self, omega_meas_rad_s: float, torque_applied_nm: float, dt_s: float) -> float:
        # 1次LPFの離散化 (指数形式。g*dt が大きくても安定)
        alpha = 1.0 - math.exp(-self.cutoff_rad_s * dt_s)
        self._omega_lpf += alpha * (omega_meas_rad_s - self._omega_lpf)
        self._torque_lpf += alpha * (torque_applied_nm - self._torque_lpf)

        d_hat = (
            self.j_nom_kgm2 * self.cutoff_rad_s * (omega_meas_rad_s - self._omega_lpf)
            + self.b_nom_nms * self._omega_lpf
            - self._torque_lpf
        )
        self.d_hat_nm = max(-self.limit_nm, min(self.limit_nm, d_hat))
        return self.d_hat_nm
