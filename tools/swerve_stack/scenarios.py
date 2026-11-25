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
    "直進": dict(ref=make_ref(3.0, 0.0, 3.5), duration=4.5, slope=None),
    "旋回移動": dict(ref=make_ref(3.0, math.pi, 4.0), duration=5.0, slope=None),
    "坂": dict(ref=make_ref(4.0, 0.0, 4.0), duration=5.0,
              slope=lambda x: SLOPE_RAD if SLOPE_X[0] <= x <= SLOPE_X[1] else 0.0),
}
VARIANTS = {
    "PID": dict(use_ff=False, use_dob=False),
    "PID+FF": dict(use_ff=True, use_dob=False),
    "PID+FF+DOB": dict(use_ff=True, use_dob=True),
}


def run(scenario: dict, use_ff: bool, use_dob: bool, steer_aware_mppi: bool = True, seed: int = 0,
        imu_slope: bool = True):
    params = PlantParams()
    plant = SwervePlant(params, scenario["slope"])
    ctrl = SwerveController(params.module_positions_m, params.steer_limit_rad, scenario["ref"],
                            NominalModel(), use_ff, use_dob, steer_aware_mppi, mppi_seed=seed)
    sim_dt = CONTROL_DT_S / SIM_SUBSTEPS
    n = round(scenario["duration"] / CONTROL_DT_S)
    keys = ("t", "x", "y", "psi", "pos_err", "psi_err", "wheel_err", "steer_err",
            "u_corr", "delta0", "d_hat_drive0", "on_slope")
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


def print_table(title, rows):
    print(f"\n■ {title}  (シード{len(SEEDS)}本の平均。車輪速度は rad/s、坂ではピークを坂区間で測る)")
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


if __name__ == "__main__":
    main()
