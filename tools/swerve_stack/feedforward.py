"""モデルベース・フィードフォワード（FF）.

目標の速度・加速度から、公称モデルが必要とするトルクを先に計算して足す。
    torque_ff = J_n * alpha_ref + B_n * omega_ref + tau_c * tanh(omega_ref / eps)

* J_n * alpha_ref : 加速に必要なトルク（立ち上がりの遅れを消す。効果が最大）
* B_n * omega_ref : 粘性摩擦の分（定常偏差を減らし、I項の負担を減らす）
* tau_c * tanh()  : クーロン摩擦（低速のカクつき・起動の渋さを消す）
                    sign() は 0 付近でチャタるので tanh で滑らかにしてある

FF は入力に足すだけなので、フィードバックの極を動かさない。
FF が間違っていても PID が埋めるだけで、発振しない。
"""

import math


class MotorFeedforward:
    def __init__(
        self,
        j_nom_kgm2: float,
        b_nom_nms: float,
        coulomb_nm: float = 0.0,
        coulomb_smooth_rad_s: float = 0.05,
    ):
        self.j_nom_kgm2 = j_nom_kgm2
        self.b_nom_nms = b_nom_nms
        self.coulomb_nm = coulomb_nm
        self.coulomb_smooth_rad_s = coulomb_smooth_rad_s

    def torque_nm(self, omega_ref_rad_s: float, alpha_ref_rad_s2: float) -> float:
        return (
            self.j_nom_kgm2 * alpha_ref_rad_s2
            + self.b_nom_nms * omega_ref_rad_s
            + self.coulomb_nm * math.tanh(omega_ref_rad_s / self.coulomb_smooth_rad_s)
        )
