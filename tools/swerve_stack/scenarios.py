"""直進・旋回移動・坂で、下位制御の構成を比べる.

  PID          : 各軸 PID のみ
  PID+FF       : + フィードフォワード（慣性・粘性・坂の重力. 公称モデルで計算）
  PID+FF+DOB   : + 外乱オブザーバ
旋回移動では、MPPI がステア角を予測に入れない場合も比べる。
MPPI は乱数を使うので、シード3本の平均で比べる。

真のプラントと公称モデルのずれ（DOB が効くかを見るために意図的に入れている）
  質量 25 kg vs 公称 20 kg,  車輪慣性 1.33 倍,  ステア慣性 1.5 倍,
  ステアの摩擦 0.05 Nm と横力の戻しモーメントは公称モデルに無い.
測定は理想（ノイズ・量子化なし）。自己位置も真値を使う。

python3 scenarios.py profile で、速度整形の「台形」と「S 字」(docs/scurve_prompt.md §5) を比べる。
揺れはプラントに無いので、機体の加速度で揺らす 1 自由度の揺れ (固有振動数 SWAY_FN_HZ、
減衰比 SWAY_ZETA) を後から計算し、止まった後の残留振動の振幅で比べる。
"""

import math
import sys

import numpy as np

from plant import PlantParams, SwervePlant
from stack import NominalModel, SwerveController

CONTROL_DT_S = 0.001
SIM_SUBSTEPS = 5
SEEDS = (0, 1, 2)


def quintic(tau: float):
    """0..1 の時間正規化 -> (s, ds/dtau). 始点と終点で速度・加速度が0."""
    t = min(max(tau, 0.0), 1.0)
    s = 10 * t**3 - 15 * t**4 + 6 * t**5
    ds = 30 * t**2 - 60 * t**3 + 30 * t**4 if 0.0 < tau < 1.0 else 0.0
    return s, ds


def make_ref(goal_x: float, goal_psi: float, move_time_s: float):
    """x 方向へ goal_x 進みながら、姿勢を goal_psi まで回す. 戻り値は (x, y, psi, vx_b, vy_b, omega)."""
    def ref(t: float):
        s, ds = quintic(t / move_time_s)
        x, psi = goal_x * s, goal_psi * s
        vx_w = goal_x * ds / move_time_s
        om = goal_psi * ds / move_time_s
        c, sn = math.cos(psi), math.sin(psi)
        return (x, 0.0, psi, c * vx_w, -sn * vx_w, om)
    return ref


SLOPE_RAD = math.radians(10.0)
SLOPE_X = (1.2, 2.8)
SCENARIOS = {
    "直進": dict(ref=make_ref(3.0, 0.0, 3.5), duration=4.5, slope=None, move_time=3.5),
    "旋回移動": dict(ref=make_ref(3.0, math.pi, 4.0), duration=5.0, slope=None, move_time=4.0),
    "坂": dict(ref=make_ref(4.0, 0.0, 4.0), duration=5.0, move_time=4.0,
              slope=lambda x: SLOPE_RAD if SLOPE_X[0] <= x <= SLOPE_X[1] else 0.0),
}
VARIANTS = {
    "PID": dict(use_ff=False, use_dob=False),
    "PID+FF": dict(use_ff=True, use_dob=False),
    "PID+FF+DOB": dict(use_ff=True, use_dob=True),
}


def run(scenario: dict, use_ff: bool, use_dob: bool, steer_aware_mppi: bool = True, seed: int = 0,
        imu_slope: bool = True, profile: str = "trapezoid", jerk_norm_per_s: float = 5.0,
        steer_jerk_rad_s3=None):
    params = PlantParams()
    plant = SwervePlant(params, scenario["slope"])
    ctrl = SwerveController(params.module_positions_m, params.steer_limit_rad, scenario["ref"],
                            NominalModel(), use_ff, use_dob, steer_aware_mppi, mppi_seed=seed,
                            profile=profile, jerk_norm_per_s=jerk_norm_per_s,
                            steer_jerk_rad_s3=steer_jerk_rad_s3)
    sim_dt = CONTROL_DT_S / SIM_SUBSTEPS
    n = round(scenario["duration"] / CONTROL_DT_S)
    keys = ("t", "x", "y", "psi", "pos_err", "psi_err", "wheel_err", "steer_err",
            "u_corr", "delta0", "d_hat_drive0", "on_slope", "vx_w", "vy_w")
    log = {k: np.zeros(n) for k in keys}
    travel = 0.0
    prev_delta = plant.delta.copy()
    for k in range(n):
        t = k * CONTROL_DT_S
        log["on_slope"][k] = plant.slope_rad() != 0.0
        tau_s, tau_w = ctrl.update(t, CONTROL_DT_S, plant.pos.copy(), plant.psi, plant.body_twist(),
                                   plant.delta.copy(), plant.ddelta.copy(), plant.wheel_speed.copy(),
                                   plant.slope_rad() if imu_slope else 0.0)
        for _ in range(SIM_SUBSTEPS):
            plant.step(tau_s, tau_w, sim_dt)

        rx, ry, rpsi, *rv = scenario["ref"](t + CONTROL_DT_S)
        log["t"][k] = t
        log["x"][k], log["y"][k], log["psi"][k] = plant.pos[0], plant.pos[1], plant.psi
        log["vx_w"][k], log["vy_w"][k] = plant.vel[0], plant.vel[1]
        log["pos_err"][k] = math.hypot(rx - plant.pos[0], ry - plant.pos[1])
        log["psi_err"][k] = math.atan2(math.sin(plant.psi - rpsi), math.cos(plant.psi - rpsi))
        omega_ref = np.array([c.omega_ref for c in ctrl.drive])
        log["wheel_err"][k] = np.sqrt(np.mean((omega_ref - plant.wheel_speed) ** 2))
        theta_ref = np.array([c.debug.get("theta_ref", 0.0) for c in ctrl.steer])
        log["steer_err"][k] = np.sqrt(np.mean((theta_ref - plant.delta) ** 2))
        log["u_corr"][k] = np.linalg.norm(ctrl.u_cmd - np.array(rv))   # MPPI が参照速度からどれだけ補正したか
        log["delta0"][k] = plant.delta[0]
        log["d_hat_drive0"][k] = ctrl.log_d_hat_drive[0]
        travel += np.sum(np.abs(plant.delta - prev_delta))
        prev_delta = plant.delta.copy()
    log["steer_travel"] = travel
    return log


def metrics(log):
    rms = lambda a: float(np.sqrt(np.mean(a ** 2)))
    slope = log["on_slope"] > 0
    wheel_peak = float(np.max(log["wheel_err"][slope])) if slope.any() else float(np.max(log["wheel_err"]))
    return {
        "位置RMS mm": rms(log["pos_err"]) * 1e3,
        "位置最大 mm": float(np.max(log["pos_err"])) * 1e3,
        "姿勢RMS deg": math.degrees(rms(log["psi_err"])),
        "車輪速度RMS": rms(log["wheel_err"]),
        "車輪速度ピーク": wheel_peak,
        "ステア追従 mrad": rms(log["steer_err"]) * 1e3,
        "MPPI補正 m/s": float(np.mean(log["u_corr"])),
        "ステア総移動 rad": log["steer_travel"],
    }


def averaged(scenario, **kw):
    ms = [metrics(run(scenario, seed=s, **kw)) for s in SEEDS]
    return {k: float(np.mean([m[k] for m in ms])) for k in ms[0]}


def print_table(title, rows, note="車輪速度は rad/s、坂ではピークを坂区間で測る"):
    print(f"\n■ {title}  (シード{len(SEEDS)}本の平均。{note})")
    keys = list(next(iter(rows.values())).keys())
    print(f"{'構成':<22}" + "".join(f"{k:>14}" for k in keys))
    for name, m in rows.items():
        print(f"{name:<22}" + "".join(f"{m[k]:>14.2f}" for k in keys))


def main():
    results = {}
    for sname, sc in SCENARIOS.items():
        rows = {}
        for vname, v in VARIANTS.items():
            rows[vname] = averaged(sc, **v)
            print(f"  done: {sname} / {vname}", file=sys.stderr)
        if sname == "旋回移動":
            rows["同上, MPPIステア非考慮"] = averaged(sc, use_ff=True, use_dob=True, steer_aware_mppi=False)
        if sname == "坂":
            rows["PID+FF+DOB, IMUなし"] = averaged(sc, use_ff=True, use_dob=True, imu_slope=False)
        print_table(sname, rows)
        results[sname] = rows
    return results


# ---------------------------------------------------------------------------
# 台形と S 字の比較 (docs/scurve_prompt.md §5「シミュレーション比較」)
# ---------------------------------------------------------------------------

SWAY_FN_HZ = 5.0      # 揺れの固有振動数 [Hz]。S 字の J_n (= J/A) の既定 5 /s と合わせてある
SWAY_ZETA = 0.02      # 揺れの減衰比 (腕・ブロックはほとんど減衰しない想定)
JERK_SAMPLE_S = 0.01  # 機体ジャークを測る刻み [s] (1 kHz のままだとタイヤ力の細かい振動を拾う)
STEER_JERK_RAD_S3 = 1600.0   # ステアの S 字の J (立ち上げ 80/1600 = 0.05 s)
PROFILES = {
    "台形": dict(profile="trapezoid"),
    "S字 (機体)": dict(profile="scurve"),
    "S字 (機体+ステア)": dict(profile="scurve", steer_jerk_rad_s3=STEER_JERK_RAD_S3),
}


def sway(log, natural_frequency_hz=SWAY_FN_HZ, zeta=SWAY_ZETA):
    """機体の加速度 (world) で揺らした 1 自由度の揺れの振幅 [m] の時系列.

    x'' + 2 zeta w x' + w^2 x = -a_body  を x, y で別々に解き、振幅 sqrt(x^2 + (x'/w)^2) を合わせる。
    """
    w = 2.0 * math.pi * natural_frequency_hz
    dt = CONTROL_DT_S
    amp = np.zeros(len(log["t"]))
    for axis in ("vx_w", "vy_w"):
        accel = np.diff(log[axis], prepend=0.0) / dt
        x = xd = 0.0
        out = np.zeros_like(accel)
        for k, a in enumerate(accel):
            xd += (-a - 2.0 * zeta * w * xd - w * w * x) * dt
            x += xd * dt
            out[k] = x * x + (xd / w) ** 2
        amp += out
    return np.sqrt(amp)


def profile_metrics(log, scenario):
    rms = lambda a: float(np.sqrt(np.mean(a ** 2)))
    t = log["t"]
    gx, gy, gpsi, *_ = scenario["ref"](1e9)       # 最終の目標
    dist = np.hypot(log["x"] - gx, log["y"] - gy)
    dpsi = np.abs(np.arctan2(np.sin(log["psi"] - gpsi), np.cos(log["psi"] - gpsi)))
    away = np.nonzero((dist >= 0.02) | (dpsi >= math.radians(2.0)))[0]   # 到着の許容 (stack.py と同じ)
    arrive_k = away[-1] + 1 if away.size else 0
    arrived = arrive_k < len(t)
    stride = round(JERK_SAMPLE_S / CONTROL_DT_S)
    v = np.stack([log["vx_w"][::stride], log["vy_w"][::stride]], axis=1)
    jerk = np.linalg.norm(np.diff(v, n=2, axis=0), axis=1) / JERK_SAMPLE_S ** 2
    amp = sway(log)
    after = max(arrive_k, round(scenario["move_time"] / CONTROL_DT_S))
    return {
        "位置RMS mm": rms(log["pos_err"]) * 1e3,
        "所要時間 s": float(t[arrive_k]) if arrived else float("nan"),
        "ジャークRMS": rms(jerk),
        "ジャーク最大": float(np.max(jerk)),
        "ステア総移動 rad": log["steer_travel"],
        "残留振動 mm": float(np.max(amp[after:])) * 1e3 if after < len(t) else float("nan"),
    }


def averaged_profile(scenario, **kw):
    ms = [profile_metrics(run(scenario, use_ff=True, use_dob=True, seed=s, **kw), scenario) for s in SEEDS]
    return {k: float(np.mean([m[k] for m in ms])) for k in ms[0]}


def compare_profiles():
    print(f"機体ジャークは m/s^3 ({JERK_SAMPLE_S * 1e3:.0f} ms 刻みの速度の2階差分)。"
          f"残留振動は f_n = {SWAY_FN_HZ} Hz・減衰比 {SWAY_ZETA} の揺れの、到着後の最大振幅。"
          "下位制御はすべて PID+FF+DOB。")
    results = {}
    for sname, sc in SCENARIOS.items():
        rows = {}
        for pname, kw in PROFILES.items():
            rows[pname] = averaged_profile(sc, **kw)
            print(f"  done: {sname} / {pname}", file=sys.stderr)
        print_table(sname, rows, note="所要時間は到着しなかったシードがあると nan")
        results[sname] = rows
    # J_n を振って、残留振動が J_n = f_n で最小になるか (直進)
    rows = {f"S字 J_n={jn:g}": averaged_profile(SCENARIOS["直進"], profile="scurve", jerk_norm_per_s=jn)
            for jn in (2.5, 5.0, 10.0, 20.0)}
    print_table(f"直進、J_n を変える (揺れ f_n = {SWAY_FN_HZ} Hz)", rows, note="J_n = f_n (T_j = 周期) と f_n/2 (T_j = 周期の2倍) が揺れに合う")
    results["J_n"] = rows
    return results


if __name__ == "__main__":
    if sys.argv[1:] == ["profile"]:
        compare_profiles()
    else:
        main()
