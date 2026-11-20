# br_swervedrive

`br_swervedrive` は BR 担当の本人のパッケージなので、本人の指示なしに名前も中身も変えない (本人の指示で `br_drive` → `br_swervedrive` に改名、2026-11-20)。

## ノード構成 (`br_swervedrive`、BR)

BR の独ステ1ユニットを PS4 で動かす**1ノードだけ**のパッケージ (`infinite_ctrl_node`)。
`tr_dokusute` と違い、入力・計算・機構を1ノードにまとめている (試験用。本人のパッケージなので分けない)。
仕様の正本は `br_swervedrive/src/infinite_ctrl_node.cpp` の先頭のコメント。定数 (歯車の比・速さ・見張りのしきい値など) は
`br_swervedrive/include/br_swervedrive/infinite_ctrl_params.hpp` にまとめてある (変えたらビルドし直す)。ここは要点だけ。

| 項目 | 内容 |
|---|---|
| 操舵 | DM2325 (モータ1)、歯車 13:49 (`kSteerGear`)。**速度制御 + ノードの中で位置のP制御** (`kSteerMode = kVelocity`)。何周でも回れる無限旋回で、車輪の逆転はしない。`kAngle` (ESCの角度制御、±175°、届かない向きは車輪を逆転) にも切り替えられる |
| 駆動 | DM-S3519-1EC (モータ2、中に減速機 3591/187 ≒ 19.2)、外の歯車 23:33 (`kDriveGear`)、車輪の直径 80 mm。速度制御 |
| 速さ | 最高 1.5 m/s・加速度 1.5 m/s² で決め、モータの rad/s に直す。ただし `kDriverLimitRadps` (今は 25.0 rad/s ≒ 0.70 m/s) で頭打ち。操舵はドライバの上限 45.6 rad/s の 50% (`kSteerSpeedRatio`) |
| 入出力 | `damiao/motor{1,2}/cmd`・`/state` (`damiao_ctrl` の `damiao_driver_node`)、`joy`、`serial_rx_201` の23番 (原点センサー) |
| パラメータ | `use_drive` (既定 true、false = 操舵だけ)、`origin_offset_deg` (既定 0)、`drive_coupling` (既定 −1.0) |

```
joy ─→ infinite_ctrl_node ─ damiao/motor1/cmd (操舵) ─→ damiao_driver_node ⇄ ros2can(201) ⇄ ESP32 ⇄ DM2325 / DM-S3519
            ↑   └────────── damiao/motor2/cmd (駆動) ─┘          │
            ├── damiao/motor{1,2}/state ──────────────────────────┘
            └── serial_rx_201[23] (原点センサー C-MSX951N、1 = 空 / 2 = 原点)
```

- 起動 (ros2can を別ターミナルで先に起動。`use_joy:=true` は付けない = ps4_teleop_node と取り合わないため):
  ```bash
  ros2 launch damiao_ctrl damiao_ctrl.launch.py
  ros2 run joy joy_node
  ros2 run br_swervedrive infinite_ctrl_node        # 操舵だけ: --ros-args -p use_drive:=false
  ```
- 操作: R1 を押している間だけ動く (離すと2台とも脱力)。左スティック = 車輪を向ける方向 (上 = 原点の向き)。
  R2 = 駆動の速さ (スティックを倒していなければ今の向きのまま駆動)。Options = E-STOP。△ = 原点合わせ。
  起動後に最初に R1 を押した時に、自動で1回原点合わせをする
- **操舵と駆動のつながり**: 駆動の軸が操舵の回転軸の中を通っている (DM-S3519 → 上の歯車 1:1 → 縦の軸 → 傘歯車 → 23:33 → 車輪)
  ので、操舵だけ回しても車輪が回る。駆動モータに「車輪の向きが変わる速さ × `drive_coupling`」を足して打ち消す。
  −1.0 は 2026-11-19 に止まった状態で目で確認した。駆動中の確認は、一輪なのでまだしていない
- 見張り (どれも2台とも脱力してノードを終える):
  - 固まり: 駆動のトルク 3 N·m 以上で速度 0.5 rad/s 未満が 0.5 秒 (2026-11-17 の発煙の対策)
  - 暴走: 指令と逆向きに 10 rad/s 以上ずれる、または上限 +5 rad/s 超えが 0.1 秒
  - 操舵の位置の数え間違い・位置の読みの止まり、故障・返事の途絶、joy の途絶 (0.5 秒) や送り手が2つ以上
- 起動時は、帰還が途切れずに 0.5 秒続いてから位置を確定する。24V をノードより後に入れた時に、
  位置の飛びと見誤って止まっていたため (2026-11-19)
- 原点合わせの繰り返しの精度 (2026-11-19、28回): ±0.3° 以内が19回、最大 −1.72°、中央値は約 0.1°
- 駆動の位置の帰還は**中の回転子の値**で返ってくる (速度は出口の値)。ログでは `kDriveEncRatio` で割って出口の値に直している。
  制御には使っていない
- 新しい ESC のコイル温度 (約 195 °C) はおかしな値なので、MOS 温度を見る
- `zakinema.h` / `matrix.h` (4輪の運動学) はライブラリ `br_swervedrive_lib` として書き出しているが、今の1輪のノードでは使っていない
- 速度の上限は3か所にある。上げる時はそろえる:
  1. ESP32 のファーム: ros2can の `feature/damiao-per-motor-maxvel` の表、駆動は 40 rad/s
  2. `~/rrst-ros2-ws/damiao_ctrl/config/damiao_ctrl.yaml` の `max_vel_per_motor`: [45.6, 40.0]
     (2026-11-19 に 20 から 40 へ。taiyouss のコミット 7cf6d51)
  3. ノードの `kDriverLimitRadps`: 25.0
  
  25 → 30 → 35 → 40 と段階的に上げる予定

## コマンド

```bash
# br_swervedrive (ビルドは Jazzy だけで通る。動かすには ~/rrst-ros2-ws の damiao_ctrl が要る)
source ~/rrst-ros2-ws/install/setup.bash && colcon build --packages-select br_swervedrive
source install/setup.bash
```
