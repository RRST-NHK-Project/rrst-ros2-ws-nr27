"""PID 制御器（位置ループ用）.

入力  : 偏差 error, 偏差の時間微分 error_rate, 制御周期 dt
出力  : 制御量（トルク指令など。飽和は呼び出し側で行う）

微分項は「偏差を数値微分」せず、呼び出し側が持っている速度情報
(目標速度 - 測定速度) をそのまま受け取る。数値微分によるノイズ増幅と
目標のステップ変化によるキック (derivative kick) を避けるため。
"""


class PID:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        # ki を掛けた後の値で持つ。実行中に ki を変えても出力が跳ばない。
        self.integral_term = 0.0

    def reset(self) -> None:
        self.integral_term = 0.0

    def update(
        self,
        error: float,
        error_rate: float,
        dt_s: float,
        freeze_integral: bool = False,
    ) -> float:
        """freeze_integral=True の間は積分を止める（アンチワインドアップ）.

        止めるかどうかは、全体の指令が飽和しているかを知っている
        呼び出し側が決める。PID単体は FF や DOB の寄与を知らないため。
        """
        if not freeze_integral:
            self.integral_term += self.ki * error * dt_s
            self.integral_term = max(
                -self.integral_limit, min(self.integral_limit, self.integral_term)
            )
        return self.kp * error + self.integral_term + self.kd * error_rate
