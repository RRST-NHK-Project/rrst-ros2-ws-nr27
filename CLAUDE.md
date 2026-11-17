# CLAUDE.md

NHKロボコン2027 BR の ROS 2 (Jazzy, Ubuntu 24.04) ワークスペース。
現状のパッケージは `nr27_br_hw_ctrl` (独立ステア足回りの制御) のみ。
今は**独ステ1ユニットの動作確認**の段階。機能は最小限にとどめる。

## ノード構成 (`nr27_br_hw_ctrl`)

機能ごとにノードを分ける。ノードを足すときも、役割を混ぜない。

| 分類 | ノード | 役割 |
|---|---|---|
| 入力 | `br_joy_teleop` | PS4 → `cmd_vel` (R1を押している間だけ) |
| 計算 | `br_swerve_kinematics` | `cmd_vel` → ステア角の目標・車輪速度の目標 (有限ステア) |
| 計算 | `br_pid` | 汎用PID+フィードフォワード。launchでremapして使う |
| 計算 | `dob_node` | 外乱オブザーバ (独立。1ユニット試験では使わない。`odom`が要る) |
| 機構 | `br_steer_dm2325` | DM2325 ステア。目標角 → ros2can |
| 機構 | `br_drive_735` | マブチ735 駆動。duty → ros2can、エンコーダから速度 |

```
joy → cmd_vel → br_swerve_kinematics ─ steer/angle_target → br_steer_dm2325 ⇄ ros2can(201)
                     ↑ steer/angle ───────────────────────────────┘
                     └ drive/speed_target → br_pid → drive/duty → br_drive_735 ⇄ ros2can(駆動)
                                              ↑ drive/speed ───────────┘
```

- 起動は `ros2 launch nr27_br_hw_ctrl unit_test.launch.py` (ros2can を別ターミナルで先に起動)
- 設定は `config/br_hw_ctrl.yaml` (ノード名がキー)
- 計算の中身はROSに依存しない `swerve_kinematics.hpp/.cpp` に置き、gtestを書く
- **チーム共通の `common` (`rrst-ros2-ws/common`) を先に探して使う。** 自作する前に
  `common/include/common/` に無いか見ること。今使っているのは `common/PID.hpp` の `PIDController`
  (`br_pid`)。`common/common.hpp` は存在しない `IK.hpp` を include しているので使わず、
  必要なヘッダを個別に include する。`PacketController` / `Ros2CanPacketController` は
  DM2325 (MODE_DAMIAO) と MES基板のMD1+ENC1 に対応していないので、`damiao_slots.hpp` と
  各ノードのスロット番号は自前
- 複数ノードで繰り返す処理は小さなヘッダにして使い回す:
  `latest_value.hpp` (最後に受けた値と時刻。「届かなければ止める」判定)、
  `stop_on_shutdown.hpp` (終了時の停止指令)
- **機構ノードは「入力が timeout 以上届かなければ止める」を必ず持つ。**
  上流のノードが落ちても、下流が自分で止まる (停止の責任を1か所に集めない)
- ユニットを増やすとき: 今は1ユニットなので、機構ノードが `serial_tx_<ID>` の24スロット
  すべてを送っている。複数ユニットで同じIDを共有すると、互いのスロットを0で上書きする。
  その場合は、スロットをまとめて送る専用ノードを挟むこと

## コマンド

```bash
# ビルド (ワークスペースルートで)
# common (rrst-ros2-ws/common) はこのワークスペースに無いので --base-paths で足す
colcon build --base-paths . ../rrst-ros2-ws/common --packages-up-to nr27_br_hw_ctrl
source install/setup.bash

# テスト (gtest + cpplint + flake8/pep257 など)
colcon test --base-paths . ../rrst-ros2-ws/common --packages-select nr27_br_hw_ctrl
colcon test-result --verbose --test-result-base build/nr27_br_hw_ctrl

# C++ の整形 (.clang-format = Google)
clang-format -i nr27_br_hw_ctrl/src/*.cpp nr27_br_hw_ctrl/include/nr27_br_hw_ctrl/*.hpp nr27_br_hw_ctrl/test/*.cpp
```

## コーディング規約

### C++: Google C++ Style Guide に従う
https://google.github.io/styleguide/cppguide.html

- 書式は `.clang-format` (BasedOnStyle: Google) に任せる。C++を変更したら clang-format をかける
- 規約は cpplint (Google製、`colcon test` で実行) で確認する。違反を残さない
- よく効くルール:
  - 命名: 型・関数は `PascalCase`、変数は `snake_case`、クラスのメンバ変数は末尾 `_`、
    定数は `kPascalCase`、名前空間は `snake_case`
  - 1行80文字、インデント2スペース、`public:` は1スペース下げる
  - `std::bind` ではなくラムダ、`using namespace` は使わない
  - データを束ねるだけなら `struct` (メンバ末尾 `_` なし)、振る舞いを持つなら `class`
  - 関数は短く、1つの役割にする。長くなったら名前の付いた関数に分ける
- このリポジトリでの例外 (ROS 2 の慣習に合わせる):
  - 拡張子は `.hpp` / `.cpp`
  - インクルードガードは ament_cpplint の形式 `NR27_BR_HW_CTRL__FILE_NAME_HPP_`
  - 著作権表記のチェック (`legal/copyright`) は、方針が決まるまで外している
- 動作確認用の簡単なノード (例: `br_joy_teleop_node.cpp`) は、チームで読み慣れた nr26
  (`~/rrst-ros2-ws-nr26/nr26_r1_hw_ctrl/src/R1_SwerveDrive.cpp`) の書き方に合わせる:
  設定値は先頭の `#define`、joy は `ps4_listener_callback` で全ボタンを変数に並べてから使う
  (使わないものはコメントアウト)、`main` で figlet を表示。機能は必要最小限にする
- ROSに依存しない計算 (運動学など) は別ファイルに分けて gtest を書く
  (例: `swerve_kinematics.hpp` と `test/test_swerve_kinematics.cpp`)

### Python (launch ファイル)
ROS 2 の ament_flake8 / ament_pep257 に従う (シングルクォート、1行99文字、
docstring は1行目を空けて書き、要約は `.` で終える)。

### コメント
- コメントは日本語。何をしているかより「なぜそうしたか」を書く
  (過去の不具合・実測値・日付を残す)
- 物理量には単位を書く (`[m/s]`, `[rad, 出力軸]` など)
- 日本語は空白で区切られないので、`.clang-format` で `ReflowComments: false` にしている。
  コメントの改行は文の切れ目で手で入れる。行末コメントが80文字を超えるなら行の上に書く

## ハードウェア構成 (コードを読む前に知っておくこと)

- 通信はすべて ros2can (`~/ros2can`、`ros2 run ros2can ros2can --nogui`) 経由。
  `serial_tx_<DEVICE_ID>` / `serial_rx_<DEVICE_ID>` に int16 x 24 スロットを送受信する
- ステア: DAMIAO DM2325 + DM3520 (ESC)。ESP32 を ros2can の `MODE_DAMIAO` (DEVICE_ID 201) で使う。
  1台6スロット、最大4台。ESCが位置制御を回すので、PCは目標角と速度上限を送るだけ。
  CAN線はDamiao専用 (他モードとCAN IDが衝突する)。
  スロットの単位の正本は ros2can のファーム (`~/ros2can/firmware/xiao-esp32-s3_can2io/src/damiao.cpp`)。
  速度系は 0.01 rad/s/LSB、位置は 0.001 rad/LSB。
  `~/rrst-ros2-ws/damiao_ctrl` (slot_layout.hpp / README) は速度を 0.001 と書いていて古いので、
  参考にしても単位だけは写さない (写すと速度上限が10倍になる。test_damiao_slots で確認している)
- 駆動: マブチ735。CAN_HOST 配下の MES 基板 (1ノード5スロット、MD1=local 0, ENC1=local 1)。
  ENC は PCNT が ±32767 でリセットされるので `serial_rx_<ID>_unwrapped` を使う
  (MODE_DAMIAO の位置には `_unwrapped`/`_zeroed` は使えない)

## 壊しやすいポイント

- **同じ `serial_tx_<ID>` に publish するノードは1つだけにする。** メッセージは24スロット丸ごとなので、
  複数ノードが送ると互いの担当外スロットを0で上書きする (1ユニット試験では、ステアと駆動でIDが別なので問題ない)
- **終了時の停止指令は pre-shutdown コールバックで送る。** `rclcpp::on_shutdown` や spin の後の publish は
  通信が切れた後なので届かない (damiao_ctrl で2026-10-14に実機確認)
- DMモータは全ゼロ (= DISABLE) が安全な既定値。`br_steer_dm2325` は故障をラッチし、
  ノードを再起動するまで DISABLE のままにする (故障のまま有効化を繰り返さないため)
- 実機のモータを動かすコードなので、スロット割り当て・スケール・上限 (duty_max, 速度上限)・
  タイムアウト・停止処理に関わる変更は、意図しない動作につながりうることをユーザーに明示する
- 実機の値が決まっていないパラメータは `config/br_hw_ctrl.yaml` で【仮置き】と書いてある。
  勝手に確定値として扱わない

## LiDAR (`nr27_sensors`)

機種: Hokuyo UST-30LX (Ethernet 192.168.0.10)、YDLIDAR TG30 (USBシリアル)、LD19 (USBシリアル) を1台ずつ。
launch と config だけのパッケージ。ドライバは他のワークスペースに入っていて、**改変しない**:

| 機種 | ドライバ | どこ |
|---|---|---|
| LD19 | `ldlidar_component` (ldrobot-lidar-ros2) | `~/ros2_ws` |
| UST-30LX | `urg_node2` | `~/ros2_ws` |
| TG30 | `ydlidar_ros2_driver` | `~/ydlidar_ros2_ws` |

```bash
source ~/ros2_ws/install/setup.bash && source ~/ydlidar_ros2_ws/install/setup.bash
ros2 launch nr27_sensors lidar.launch.py lidars:=ld19,tg30,ust30lx   # 使うものだけ並べる
```

- **機種の違いは `nr27_sensors` の中に閉じ込める。** 出力は機種によらず
  `/<name>/scan` (LaserScan)、frame `<name>_link` (name = ld19 / tg30 / ust30lx)。
  使う側 (SLAM、壁検知、MPPI のコストマップ) はパラメータ `scan_topic` で選ぶだけにし、
  機種名やドライバのパッケージ名をコードに書かない (nr26 は `/ldlidar_node/scan` を直書きしていて、機種を増やせなかった)
- 取り付け位置の TF (`base_link` → `<name>_link`) は `config/lidar_mount.yaml` の1か所だけ。
  各 launch に `static_transform_publisher` を複製しない
- 機種を増やすときは `launch/<name>.launch.py` と `config/<name>.yaml` を足し、`lidar.launch.py` の `MODELS` に足す
- USBシリアルは `/dev/ttyUSBx` の番号で呼ばない。`udev/99-nr27-lidar.rules` で `/dev/ld19`, `/dev/tg30` に固定する
  (LD19 と TG30 は同じ CP210x の可能性があり、VID/PID では区別できない。シリアル番号かUSBポートで区別する)
- **UST-30LX は同時接続が1つだけ。** 別の `urg_node2` が動いていると `Could not open network Hokuyo` で失敗する
- ドライバ側の落とし穴 (実機で確認した):
  - ydlidar_ros2_driver の付属 launch は ROS 1 時代の引数 (`node_executable`) で Jazzy では動かない。使わず自前の launch で起動する
  - ydlidar_ros2_driver は `intensity_bit`, `debug`, `m1_mode`〜`m3_mode` を既定値なしで宣言するので、yaml に無いと異常終了する
  - LD19 の `lidar.model` は `'LD19'` (nr26 の yaml の `LDLiDAR_LD19` はこのドライバでは不可)
