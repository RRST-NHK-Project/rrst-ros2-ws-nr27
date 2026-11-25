# swerve_stack (BR の足回り制御の参照実装、Python)

有限旋回の独ステ (BR) の制御を、ROS に載せる前に Python のシミュレータで確かめるための参照実装。
ROS のパッケージではない (package.xml が無いので colcon はビルドしない)。式は `MODEL.md`。

- 上位: MPPI (`mppi.py`、25 Hz)。予測モデルにステア角を入れる
- 速度整形: 台形 (加速度だけ制限) または S 字 (ジャークも制限、`motion_profile.py`)
- 各軸: FF + PID + DOB (`steer_axis.py`・`drive_axis.py`)
- S 字の仕様は `docs/scurve_prompt.md` (§3.4 は 2026-11-24 に合意した式に変えてある)

```bash
cd tools/swerve_stack
python3 scenarios.py            # 下位制御の構成を比べる (約1分半、結果は results.txt と同じになる)
python3 scenarios.py profile    # 速度整形の台形と S 字を比べる (約3分、results_profile.txt)
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider test_motion_profile.py test_scurve_integration.py
```

必要なのは numpy・matplotlib・pytest だけ。C++ への移植 (`docs/scurve_prompt.md` の Phase 3) はまだ。

既知の問題 (2026-11-24): 旋回移動・S 字 (機体) で、到着の判定で指令を 0 にした後も S 字の減速で
機体が回り続け、姿勢の誤差が許容 (2°) を出たまま止まることがある (シード 3 本のうち 1 本)。
