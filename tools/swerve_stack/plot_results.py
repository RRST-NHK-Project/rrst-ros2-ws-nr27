"""比較結果を2枚の図にする（シード0）.

  左: 坂 - 車輪速度の追従誤差（4輪RMS）。坂区間を灰色で示す
  右: 旋回移動 - モジュール0のステア角。MPPI がステアを予測に入れる/入れない
"""

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scenarios import SCENARIOS, run

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]   # 固定順: 青, 橙, 緑青

plt.rcParams.update({
    "font.family": ["Noto Sans CJK JP", "IPAexGothic", "DejaVu Sans"],
    "axes.facecolor": SURFACE, "figure.facecolor": SURFACE,
    "axes.edgecolor": "#d9d8d4", "axes.labelcolor": INK_2,
    "xtick.color": INK_2, "ytick.color": INK_2, "text.color": INK,
    "axes.grid": True, "grid.color": "#ecebe8", "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
})


def smooth(a, n=25):
    return np.convolve(a, np.ones(n) / n, mode="same")


def main():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.2))

    sc = SCENARIOS["坂"]
    variants = [("PID", False, False), ("PID+FF", True, False), ("PID+FF+DOB", True, True)]
    for color, (name, ff, dob) in zip(SERIES, variants):
        lg = run(sc, use_ff=ff, use_dob=dob)
        y = smooth(lg["wheel_err"], 100)   # 0.1 s 移動平均
        ax1.plot(lg["t"], y, color=color, linewidth=2, label=name)
    on = lg["on_slope"] > 0
    t_on = lg["t"][on]
    ax1.axvspan(t_on[0], t_on[-1], color="#e7e6e2", zorder=0)
    ax1.text((t_on[0] + t_on[-1]) / 2, ax1.get_ylim()[1] * 0.92, "坂 10°", ha="center", color=INK_2)
    ax1.set_title("坂: 車輪速度の追従誤差", loc="left", color=INK)
    ax1.set_xlabel("時間 [s]")
    ax1.set_ylabel("4輪RMS [rad/s]（0.1 s 平均）")
    ax1.legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=3)

    sc = SCENARIOS["旋回移動"]
    for color, (name, aware) in zip(SERIES[:2], [("MPPI ステア考慮", True), ("MPPI ステア非考慮", False)]):
        lg = run(sc, use_ff=True, use_dob=True, steer_aware_mppi=aware)
        ax2.plot(lg["t"], np.degrees(lg["delta0"]), color=color, linewidth=2,
                 label=f"{name}（総移動 {lg['steer_travel']:.0f} rad）")
    for lim in (170, -170):
        ax2.axhline(lim, color=INK_2, linewidth=1, linestyle="--")
    ax2.text(0.05, 178, "可動域 ±170°", color=INK_2)
    ax2.set_ylim(-190, 200)
    ax2.set_title("旋回移動: モジュール0のステア角", loc="left", color=INK)
    ax2.set_xlabel("時間 [s]")
    ax2.set_ylabel("ステア角 [deg]")
    ax2.legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=2)

    fig.tight_layout()
    fig.savefig("results.png", dpi=140)
    print("saved results.png")


if __name__ == "__main__":
    main()
