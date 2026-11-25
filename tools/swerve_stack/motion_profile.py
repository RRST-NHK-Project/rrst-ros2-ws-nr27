"""加加速度 (ジャーク) 制限つき S 字の速度プロファイル.

docs/scurve_prompt.md の §3 の式を実装する。記号:
    A   加速度の上限 [単位/s^2]
    J   ジャークの上限 [単位/s^3]
    T_j = A / J  加速度の立ち上げ時間 [s]
単位は並進なら m、回転なら rad。どちらでも同じ式で使える。

作るもの:
    1. 計画版   plan_velocity_change / plan_rest_to_rest / plan_move
       始点と終点が分かっている移動を、事前に S 字で計画する (DOCK の寄せ、ステアの参照軌道)。
    2. 逐次版   JerkLimitedFilter
       毎周期変わる目標速度を、ジャーク制限を守って追う。
    3. 逐次版 (ベクトル)  vector_step / VectorJerkLimitedFilter
       機体速度 (vx, vy, omega) に、速度ベクトルの向きを保ったまま 2 をかける (§3.5)。
       MPPI の予測モデルも同じ vector_step を (K, 3) の配列で呼ぶ。

逐次版の判定は §3.4 の原文から変えてある (2026-11-24 合意。docs/scurve_prompt.md の §3.4 の追記)。
原文の「同じ向きで |e| <= a^2/(2J) + |a| dt なら立ち下げ」は、立ち下げを始めても次の周期に
「それ以外」へ戻って加速度を上げ直すので、行き過ぎが O(A dt)、到達が 4〜5 周期遅れになった。
ここでは毎周期、次の加速度 x を「x で 1 周期進み、そのあと J で 0 まで下げると増える速度」
    g(x) = x^2 / (2J) + |x| dt / 2
が残りの速度差 |e| にちょうど等しくなる値に選び、ジャークと加速度の上限で切る:
    x* = sign(e) J ( -dt/2 + sqrt(dt^2/4 + 2|e|/J) )
    a  = clip(clip(x*, a - J dt, a + J dt), -A, A)
g(x) の |x| dt / 2 は、v += a_new * dt で更新したときの離散の和
    sum_{k=0}^{n} (x - k J dt) dt   (n = x / (J dt))
を閉じた式にしたもの。行き過ぎは O(J dt^2)、到達は計画版から 1 周期以内になる。
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np


def jerk_from_natural_frequency(accel_max: float, natural_frequency_hz: float) -> float:
    """揺れの固有振動数 f_n から J を決める (§3.3: J = A f_n、T_j = 1 / f_n)."""
    if natural_frequency_hz <= 0.0:
        raise ValueError('natural_frequency_hz は正にする')
    return accel_max * natural_frequency_hz


# ---------------------------------------------------------------------------
# §3.1 加速の 1 区間
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class VelocityChange:
    """v0 → v1 の 1 区間の結果."""

    duration: float      # T [s]
    distance: float      # D (符号つき) [単位]
    peak_accel: float    # 加速度の最大の大きさ [単位/s^2]
    reaches_accel_max: bool  # True: ケース 1 (A に届く)、False: ケース 2


def velocity_change(v0: float, v1: float, accel_max: float, jerk_max: float) -> VelocityChange:
    """v0 → v1 に変えるのにかかる時間と距離 (§3.1)。減速は同じ形を符号反転して扱う."""
    _check_limits(accel_max, jerk_max)
    dv = abs(v1 - v0)
    a, j = accel_max, jerk_max
    if dv >= a * a / j:
        # ケース 1: 加速度が A に届く
        duration = dv / a + a / j
        distance = (v0 + v1) / 2.0 * duration
        return VelocityChange(duration, distance, a, True)
    # ケース 2: A に届かない。ピーク加速度は sqrt(J dv)
    duration = 2.0 * math.sqrt(dv / j)
    distance = (v0 + v1) * math.sqrt(dv / j)
    return VelocityChange(duration, distance, math.sqrt(j * dv), False)


# ---------------------------------------------------------------------------
# 計画版: ジャーク一定の区間を並べたもの
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class State:
    """1 次元の運動の状態."""

    position: float = 0.0
    velocity: float = 0.0
    accel: float = 0.0
    jerk: float = 0.0


def _advance(s: State, jerk: float, t: float) -> State:
    """ジャーク一定で t 秒進めた状態 (多項式で厳密に積分)."""
    return State(
        position=s.position + s.velocity * t + s.accel * t * t / 2.0 + jerk * t ** 3 / 6.0,
        velocity=s.velocity + s.accel * t + jerk * t * t / 2.0,
        accel=s.accel + jerk * t,
        jerk=jerk,
    )


@dataclass
class SCurvePlan:
    """ジャーク一定の区間 (時間, ジャーク) の列で表した計画."""

    start: State
    phases: List[Tuple[float, float]] = field(default_factory=list)

    @property
    def duration(self) -> float:
        """計画の所要時間 [s]."""
        return sum(t for t, _ in self.phases)

    def sample(self, t: float) -> State:
        """時刻 t [s] の状態。t < 0 は始点、所要時間を過ぎたら終点で止まったものとする."""
        s = self.start
        if t <= 0.0:
            first_jerk = self.phases[0][1] if self.phases else 0.0
            return State(s.position, s.velocity, s.accel, first_jerk)
        for length, jerk in self.phases:
            if t <= length:
                return _advance(s, jerk, t)
            s = _advance(s, jerk, length)
            t -= length
        # 終わった後は等速 (加速度は終点の値。正しい計画なら 0)
        return State(s.position + s.velocity * t, s.velocity, 0.0, 0.0)

    def end(self) -> State:
        """終点の状態 (ジャークは 0 とする)."""
        s = self.start
        for length, jerk in self.phases:
            s = _advance(s, jerk, length)
        return State(s.position, s.velocity, s.accel, 0.0)


def _velocity_change_phases(v0: float, v1: float, accel_max: float,
                            jerk_max: float) -> List[Tuple[float, float]]:
    """v0 → v1 のジャークの区間 (§3.1 の ①②③)."""
    dv = v1 - v0
    if dv == 0.0:
        return []
    sign = 1.0 if dv > 0.0 else -1.0
    change = velocity_change(v0, v1, accel_max, jerk_max)
    if change.reaches_accel_max:
        t_j = accel_max / jerk_max
        t_a = abs(dv) / accel_max - accel_max / jerk_max
        return [(t_j, sign * jerk_max), (t_a, 0.0), (t_j, -sign * jerk_max)]
    t_j = math.sqrt(abs(dv) / jerk_max)
    return [(t_j, sign * jerk_max), (t_j, -sign * jerk_max)]


def plan_velocity_change(v0: float, v1: float, accel_max: float, jerk_max: float,
                         position: float = 0.0) -> SCurvePlan:
    """止まっていない始点・終点も含めて、加速度 0 → 0 で速度を v0 → v1 に変える計画."""
    _check_limits(accel_max, jerk_max)
    return SCurvePlan(State(position, v0, 0.0, 0.0),
                      _velocity_change_phases(v0, v1, accel_max, jerk_max))


# ---------------------------------------------------------------------------
# §3.2 止まった状態から止まるまで
# ---------------------------------------------------------------------------

def peak_velocity_rest_to_rest(distance: float, accel_max: float, jerk_max: float) -> float:
    """速度の上限が無いとき、距離 L を止まって止まる移動のピーク速度 (§3.2)。L の符号は無視する.

    ケース 1 の式で計算し、v_p が A^2/J 未満ならケース 2 で計算し直す (§3.2 の順番)。
    """
    _check_limits(accel_max, jerk_max)
    length = abs(distance)
    a, j = accel_max, jerk_max
    v_p = (a / 2.0) * (-a / j + math.sqrt((a / j) ** 2 + 4.0 * length / a))
    if v_p < a * a / j:
        v_p = (j * length * length / 4.0) ** (1.0 / 3.0)
    return v_p


def plan_rest_to_rest(distance: float, vel_max: float, accel_max: float, jerk_max: float,
                      position: float = 0.0) -> SCurvePlan:
    """止まった状態から distance だけ進んで止まる計画 (§3.2)。distance は負でもよい."""
    _check_limits(accel_max, jerk_max)
    if vel_max <= 0.0:
        raise ValueError('vel_max は正にする')
    sign = 1.0 if distance >= 0.0 else -1.0
    length = abs(distance)
    if length == 0.0:
        return SCurvePlan(State(position, 0.0, 0.0, 0.0), [])
    accel_distance = velocity_change(0.0, vel_max, accel_max, jerk_max).distance
    if length >= 2.0 * accel_distance:
        # 最高速に届く: 途中に等速区間が入る。T = L/v_max + v_max/A + A/J (ケース 1 のとき)
        v_p = vel_max
        cruise = (length - 2.0 * accel_distance) / vel_max
    else:
        v_p = peak_velocity_rest_to_rest(length, accel_max, jerk_max)
        cruise = 0.0
    phases = _velocity_change_phases(0.0, sign * v_p, accel_max, jerk_max)
    if cruise > 0.0:
        phases.append((cruise, 0.0))
    phases += _velocity_change_phases(sign * v_p, 0.0, accel_max, jerk_max)
    return SCurvePlan(State(position, 0.0, 0.0, 0.0), phases)


def plan_move(distance: float, v0: float, v_end: float, vel_max: float, accel_max: float,
              jerk_max: float, position: float = 0.0, iterations: int = 100) -> SCurvePlan:
    """初速・終速が 0 でない一般の移動。ピーク速度を二分法で求める (§3.2 の最後).

    進む向きを正とし、0 <= v0, v_end <= vel_max とする (逆走して戻る計画は作らない)。
    distance が v0 → v_end に変えるだけで必要な距離より短いときは ValueError。
    """
    _check_limits(accel_max, jerk_max)
    sign = 1.0 if distance >= 0.0 else -1.0
    length = abs(distance)
    v0_f, v_end_f = sign * v0, sign * v_end  # 進む向きに直した速度
    if min(v0_f, v_end_f) < 0.0 or max(v0_f, v_end_f) > vel_max:
        raise ValueError('v0・v_end は進む向きに 0 以上 vel_max 以下にする')

    def needed(v_p: float) -> float:
        return (velocity_change(v0_f, v_p, accel_max, jerk_max).distance
                + velocity_change(v_p, v_end_f, accel_max, jerk_max).distance)

    low = max(v0_f, v_end_f)
    if needed(low) > length + 1e-12:
        raise ValueError('距離が短すぎて、この加速度・ジャークでは v_end にできない')
    if needed(vel_max) <= length:
        v_p = vel_max
        cruise = (length - needed(vel_max)) / vel_max
    else:
        high = vel_max
        for _ in range(iterations):
            mid = 0.5 * (low + high)
            if needed(mid) <= length:
                low = mid
            else:
                high = mid
        v_p = low
        cruise = (length - needed(v_p)) / v_p if v_p > 0.0 else 0.0
    phases = _velocity_change_phases(sign * v0_f, sign * v_p, accel_max, jerk_max)
    if cruise > 0.0:
        phases.append((cruise, 0.0))
    phases += _velocity_change_phases(sign * v_p, sign * v_end_f, accel_max, jerk_max)
    return SCurvePlan(State(position, sign * v0_f, 0.0, 0.0), phases)


def stopping_speed(distance: float, accel_max: float, jerk_max: float) -> float:
    """加速度 0 から、距離 distance でちょうど止まれる速度 (velocity_change(v, 0) の逆関数).

    台形の sqrt(2 A d) の S 字版。ステアの参照軌道の「この距離で止まれる速度」に使う。
        ケース 1 (v >= A^2/J): d = v^2/(2A) + v A/(2J)  →  v = (-A^2/J + sqrt(A^4/J^2 + 8 A d)) / 2
        ケース 2 (v <  A^2/J): d = v sqrt(v/J)          →  v = (J d^2)^(1/3)
    ケース 1 で計算し、A^2/J 未満ならケース 2 で計算し直す (§3.2 と同じ順番)。
    """
    _check_limits(accel_max, jerk_max)
    d = abs(distance)
    a, j = accel_max, jerk_max
    v = (-a * a / j + math.sqrt(a ** 4 / (j * j) + 8.0 * a * d)) / 2.0
    if v < a * a / j:
        v = (j * d * d) ** (1.0 / 3.0)
    return v


# ---------------------------------------------------------------------------
# §3.4 逐次版 (1 次元)。判定は冒頭の docstring の合意した形
# ---------------------------------------------------------------------------

def next_accel(error: float, accel: float, accel_max: float, jerk_max: float,
               dt: float) -> float:
    """残りの速度差 error と今の加速度 accel から、次の周期の加速度を決める.

    次の加速度 x で 1 周期進み、その後 J で 0 まで下げたときに増える速度が |error| に
    等しくなる x* を解き、ジャーク (|x - accel| <= J dt) と加速度 (|x| <= A) の上限で切る。
    """
    sign = 1.0 if error >= 0.0 else -1.0
    ideal = sign * jerk_max * (-dt / 2.0 + math.sqrt(dt * dt / 4.0 + 2.0 * abs(error) / jerk_max))
    x = min(max(ideal, accel - jerk_max * dt), accel + jerk_max * dt)
    return min(max(x, -accel_max), accel_max)


class JerkLimitedFilter:
    """目標速度を、速度・加速度・ジャークの上限を守って追う (逐次版、1 次元).

    update(v_target, dt) を毎周期呼ぶ。返す加速度はそのまま FF の加速度の参照に使う
    (§4: 差分で計算し直さない)。
    """

    def __init__(self, accel_max: float, jerk_max: float, vel_max: float = math.inf):
        _check_limits(accel_max, jerk_max)
        if vel_max <= 0.0:
            raise ValueError('vel_max は正にする')
        self.accel_max = accel_max
        self.jerk_max = jerk_max
        self.vel_max = vel_max
        self.velocity = 0.0
        self.accel = 0.0

    def reset(self, velocity: float = 0.0, accel: float = 0.0) -> None:
        """状態を置き直す (モードの切り替えや、実測に合わせるとき)."""
        self.velocity = velocity
        self.accel = accel

    def update(self, v_target: float, dt: float) -> Tuple[float, float]:
        """1 周期進めて (速度, 加速度) を返す."""
        v_target = min(max(v_target, -self.vel_max), self.vel_max)
        error = v_target - self.velocity
        j_dt = self.jerk_max * dt
        # 目標の近くで v と a を目標にそろえる (チャタリング防止、§3.4 の3つ目)
        if abs(error) < j_dt * dt and abs(self.accel) < j_dt:
            self.velocity = v_target
            self.accel = 0.0
            return self.velocity, self.accel
        self.accel = next_accel(error, self.accel, self.accel_max, self.jerk_max, dt)
        self.velocity += self.accel * dt
        return self.velocity, self.accel


# ---------------------------------------------------------------------------
# §3.5 逐次版 (ベクトル)
# ---------------------------------------------------------------------------

def vector_step(velocity: np.ndarray, accel: np.ndarray, target: np.ndarray, jerk_norm: float,
                dt: float) -> Tuple[np.ndarray, np.ndarray]:
    """正規化した空間 (各軸を加速度上限で割った空間、|加速度| <= 1) で 1 周期進める.

    配列の最後の軸がベクトルの成分。前の軸 (MPPI のサンプル K など) はまとめて計算する。
    jerk_norm = J_n = J / A [1/s]。J = A f_n なら J_n = f_n。

    1 次元版 (next_accel) と同じ考え方を、残りの差 e の向き u = e/|e| に当てはめる:
        s     = min(1, J_n (-dt/2 + sqrt(dt^2/4 + 2|e|/J_n)))   (u 方向に欲しい加速度の大きさ)
        da    = s u - a  を、ノルムが J_n dt 以下になるように縮める
        a'    = a + da  を、|a'| <= 1 の球に縮める (球への射影なので |a' - a| <= J_n dt は保たれる)
        v'    = v + a' dt
    a と e が同じ向きなら da も同じ向きなので、目標の向きが一定なら速度の向きは変わらない。
    目標の向きが変わったときは、a が J_n の範囲で回り込む。
    返り値は新しい (速度, 加速度)。入力の配列は書き換えない。
    """
    error = target - velocity
    e_norm = np.linalg.norm(error, axis=-1, keepdims=True)
    a_norm = np.linalg.norm(accel, axis=-1, keepdims=True)
    step = jerk_norm * dt
    # 目標の近くで v と a を目標にそろえる (§3.4 の3つ目をベクトルで)
    snap = (e_norm < step * dt) & (a_norm < step)

    wanted = np.minimum(1.0, jerk_norm * (-dt / 2.0 + np.sqrt(dt * dt / 4.0 + 2.0 * e_norm / jerk_norm)))
    direction = np.divide(error, e_norm, out=np.zeros_like(error), where=e_norm > 0.0)
    delta = wanted * direction - accel
    d_norm = np.linalg.norm(delta, axis=-1, keepdims=True)
    delta = delta * np.minimum(1.0, np.divide(step, d_norm, out=np.ones_like(d_norm), where=d_norm > 0.0))
    new_accel = accel + delta
    n_norm = np.linalg.norm(new_accel, axis=-1, keepdims=True)
    new_accel = new_accel / np.maximum(1.0, n_norm)
    new_velocity = velocity + new_accel * dt

    new_velocity = np.where(snap, target, new_velocity)
    new_accel = np.where(snap, 0.0, new_accel)
    return new_velocity, new_accel


class VectorJerkLimitedFilter:
    """機体速度 (vx, vy, omega) を、向きを保ったまま S 字で目標へ寄せる (§3.5).

    accel_max: 各軸の加速度上限 [m/s^2, m/s^2, rad/s^2]
    jerk_norm: 正規化したジャーク上限 J_n = J/A [1/s]。各軸のジャーク上限は A_i J_n になる。
    update() は元の単位の (速度, 加速度) を返す。加速度はそのまま FF の参照に使える。
    """

    def __init__(self, accel_max, jerk_norm: float):
        self.accel_max = np.asarray(accel_max, dtype=float)
        if np.any(self.accel_max <= 0.0) or jerk_norm <= 0.0:
            raise ValueError('accel_max と jerk_norm は正にする')
        self.jerk_norm = jerk_norm
        self._velocity_n = np.zeros_like(self.accel_max)
        self._accel_n = np.zeros_like(self.accel_max)

    @property
    def velocity(self) -> np.ndarray:
        """今の速度 (元の単位)."""
        return self._velocity_n * self.accel_max

    @property
    def accel(self) -> np.ndarray:
        """今の加速度 (元の単位)."""
        return self._accel_n * self.accel_max

    def reset(self, velocity=None, accel=None) -> None:
        """状態を置き直す (元の単位)."""
        zero = np.zeros_like(self.accel_max)
        self._velocity_n = (zero if velocity is None else np.asarray(velocity, float)) / self.accel_max
        self._accel_n = (zero if accel is None else np.asarray(accel, float)) / self.accel_max

    def update(self, target, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """1 周期進めて (速度, 加速度) を元の単位で返す."""
        target_n = np.asarray(target, dtype=float) / self.accel_max
        self._velocity_n, self._accel_n = vector_step(
            self._velocity_n, self._accel_n, target_n, self.jerk_norm, dt)
        return self.velocity, self.accel


def _check_limits(accel_max: float, jerk_max: float) -> None:
    if accel_max <= 0.0 or jerk_max <= 0.0:
        raise ValueError('accel_max と jerk_max は正にする')
