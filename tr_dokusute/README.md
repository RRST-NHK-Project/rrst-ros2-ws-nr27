# tr_dokusute

今は**独ステ1ユニットの動作確認**の段階。機能は最小限にとどめる。

このパッケージは `nr27_br_hw_ctrl` → `br_dokusute` (2026-11-19)
→ `tr_dokusute` (2026-11-20、機体が TR と分かったため) と改名した。

| | `tr_dokusute` (TR、silby72) | `br_swervedrive` (BR、Kaho-YAMAMOTO) |
|---|---|---|
| 駆動モータ | マブチ735 + MES基板 (コードの想定。実機で確認していない) | DM3519 (`damiao_ctrl` の `damiao_driver_node` 経由) |
| 入力 | `cmd_vel` (上位の自動走行につなげる) | PS4 を直接読む1ノード (`infinite_ctrl_node`) |
| 参考になる実機の知見 | — | 操舵と駆動のつながりの打ち消し、原点センサー、DM3519 の発煙 (11/17) と見張り。歯車比などは BR の値なので写さない |

TR の操舵の ESP32 は DEVICE_ID **205** (ros2can `generated_firmware/tr_damiao_205`、2026-11-21)。BR は 201、RobStride は 202、GM6020 は 203 を使う。
このファームは BR 用の soki_damiao_201 (ros2can のブランチ `feature/damiao-per-motor-maxvel` 版、フォトセンサ入り) の写しで、
`tr_photo_sensor` の値の決まり (帰還の23番、1 = 入光 / 2 = 遮光) もそれに合わせてある。

## ノード構成 (`tr_dokusute`)

機能ごとにノードを分ける。ノードを足すときも、役割を混ぜない。

| 分類 | ノード | 役割 |
|---|---|---|
| 入力 | `tr_joy_teleop` | PS4 → `cmd_vel` (R1を押している間だけ) |
| 計算 | `tr_swerve_kinematics` | `cmd_vel` → ステア角の目標・車輪速度の目標 (有限ステア) |
| 計算 | `tr_pid` | 汎用PID+フィードフォワード。launchでremapして使う |
| 計算 | `dob_node` | 外乱オブザーバ (独立。1ユニット試験では使わない。`odom`が要る) |
| 機構 | `tr_steer_dm2325` | DM2325 ステア。目標角 → ros2can。起動直後に自動で原点合わせ (フォトセンサのフラグ 64.01mm/直径163mm = 45° を往復で探し、中心を `flag_center_rad` の向きにする。終わるまで目標角を受け付けない。やり直しは `/steer/home`) |
| 機構 | `tr_drive_735` | マブチ735 駆動。duty → ros2can、エンコーダから速度 |
| センサ | `tr_photo_sensor` | 操舵の原点のフォトセンサ (serial_rx_205 の23番) → `photo_sensor/blocked`。確認用 |

```
joy → cmd_vel → tr_swerve_kinematics ─ steer/angle_target → tr_steer_dm2325 ⇄ ros2can(205)
                     ↑ steer/angle ───────────────────────────────┘
                     └ drive/speed_target → tr_pid → drive/duty → tr_drive_735 ⇄ ros2can(駆動)
                                              ↑ drive/speed ───────────┘
```

- 起動は `ros2 launch tr_dokusute unit_test.launch.py` (ros2can を別ターミナルで先に起動)
- 設定は `config/tr_dokusute.yaml` (ノード名がキー)
- 計算の中身はROSに依存しない `swerve_kinematics.hpp/.cpp` に置き、gtestを書く
- `common` で今使っているのは `common/PID.hpp` の `PIDController` (`tr_pid`)。
  `PacketController` / `Ros2CanPacketController` は
  DM2325 (MODE_DAMIAO) と MES基板のMD1+ENC1 に対応していないので、`damiao_slots.hpp` と
  各ノードのスロット番号は自前 (`common` を先に探す決まりは AGENTS.md)
- 複数ノードで繰り返す処理は小さなヘッダにして使い回す:
  `latest_value.hpp` (最後に受けた値と時刻。「届かなければ止める」判定)、
  `stop_on_shutdown.hpp` (終了時の停止指令)
- ユニットを増やすとき: 今は1ユニットなので、機構ノードが `serial_tx_<ID>` の24スロット
  すべてを送っている。複数ユニットで同じIDを共有すると、互いのスロットを0で上書きする。
  その場合は、スロットをまとめて送る専用ノードを挟むこと

## コマンド

```bash
# ビルド (ワークスペースルートで)
# common はこのワークスペースにコピーしてあるので --base-paths は要らない
colcon build --packages-up-to tr_dokusute
source install/setup.bash

# テスト (gtest + cpplint + flake8/pep257 など)
colcon test --packages-select tr_dokusute
colcon test-result --verbose --test-result-base build/tr_dokusute

# C++ の整形 (.clang-format = Google)
clang-format -i tr_dokusute/src/*.cpp tr_dokusute/include/tr_dokusute/*.hpp tr_dokusute/test/*.cpp
```

## コーディング規約

ここの C++ の規約は `tr_dokusute` に当てはめる。`br_swervedrive/` は自分の `.clang-format` (LLVM 系、インデント4) を持つので、この規約に合わせない。

### C++: Google C++ Style Guide に従う
https://google.github.io/styleguide/cppguide.html

- 書式はリポジトリのルートの `.clang-format` (BasedOnStyle: Google) に任せる。C++を変更したら clang-format をかける
- 規約は cpplint (Google製、`colcon test` で実行) で確認する。違反を残さない
- よく効くルール:
  - 命名: 型・関数は `PascalCase`、変数は `snake_case`、クラスのメンバ変数は末尾 `_`、
    定数は `kPascalCase`、名前空間は `snake_case`
  - 1行80文字、インデント2スペース、`public:` は1スペース下げる
  - `std::bind` ではなくラムダ、`using namespace` は使わない
  - データを束ねるだけなら `struct` (メンバ末尾 `_` なし)、振る舞いを持つなら `class`
  - 関数は短く、1つの役割にする。長くなったら名前の付いた関数に分ける
- このパッケージでの例外 (ROS 2 の慣習に合わせる):
  - 拡張子は `.hpp` / `.cpp`
  - インクルードガードは ament_cpplint の形式 `TR_DOKUSUTE__FILE_NAME_HPP_`
  - 著作権表記のチェック (`legal/copyright`) は、方針が決まるまで外している
- 動作確認用の簡単なノード (例: `tr_joy_teleop_node.cpp`) は、チームで読み慣れた nr26
  (rrst-ros2-ws-nr26 の `nr26_r1_hw_ctrl/src/R1_SwerveDrive.cpp`) の書き方に合わせる:
  設定値は先頭の `#define`、joy は `ps4_listener_callback` で全ボタンを変数に並べてから使う
  (使わないものはコメントアウト)、`main` で figlet を表示。機能は必要最小限にする

### コメント
- 日本語は空白で区切られないので、ルートの `.clang-format` で `ReflowComments: false` にしている。
  コメントの改行は文の切れ目で手で入れる。行末コメントが80文字を超えるなら行の上に書く

## ハードウェア構成 (コードを読む前に知っておくこと)

- 通信はすべて ros2can (RRST-NHK-Project/ros2can、`ros2 run ros2can ros2can --nogui`) 経由。
  `serial_tx_<DEVICE_ID>` / `serial_rx_<DEVICE_ID>` に int16 x 24 スロットを送受信する
- ステア: DAMIAO DM2325 + DM3520 (ESC)。ESP32 を ros2can の `MODE_DAMIAO` (DEVICE_ID 205) で使う。
  1台6スロット、最大4台。ESCが位置制御を回すので、PCは目標角と速度上限を送るだけ。
  CAN線はDamiao専用 (他モードとCAN IDが衝突する)。
  スロットの単位の正本は ros2can のファーム (ros2can の `firmware/xiao-esp32-s3_can2io/src/damiao.cpp`)。
  速度系は 0.01 rad/s/LSB、位置は 0.001 rad/LSB。
  rrst-ros2-ws の `main` の `damiao_ctrl` (slot_layout.hpp / README) は速度を 0.001 と書いていて古いので、
  参考にしても単位だけは写さない (写すと速度上限が10倍になる。test_damiao_slots で確認している。
  `develop` の版は 0.01 に直っている、2026-11-21 時点)
- 駆動: マブチ735。CAN_HOST 配下の MES 基板 (1ノード5スロット、MD1=local 0, ENC1=local 1)。
  ENC は PCNT が ±32767 でリセットされるので `serial_rx_<ID>_unwrapped` を使う
  (MODE_DAMIAO の位置には `_unwrapped`/`_zeroed` は使えない)
