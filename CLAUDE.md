# CLAUDE.md

NHKロボコン2027 TR の ROS 2 (Jazzy, Ubuntu 24.04) ワークスペース。
パッケージは `tr_dokusute` (独立ステア足回りの制御)・`br_swervedrive` (BR の独ステ1ユニットの試験)・`nr27_sensors` (LiDAR)・`nr27_localization` (自己位置)。
`common` は `~/rrst-ros2-ws/common` (ヘッダだけ。`PID.hpp` など) のコピー (2026-11-20)。GitHub Actions のビルドで
`rrst-ros2-ws` が無くても `tr_dokusute` が通るように入れた。元が変わったらコピーし直す。
シム・作戦・フィールドの寸法は本人の別ワークスペース `~/dev_ws` (Gazebo、ルールブック V1.1) にある。
**dev_ws は BR (完全自動) のシム。** このワークスペースの機体は TR (2026-11-20 に TR と BR を取り違えていたと分かった)。
dev_ws の作戦・L1/L2 の話をそのまま TR に当てはめない (TR はグラウンドとトランスファーまで【3.2.1】)。

**独ステのパッケージは2つあり、機体が違う。** `tr_dokusute` は TR、`br_swervedrive` (Kaho-YAMAMOTO) は BR の独ステ。

## パッケージごとの説明

構成・コマンド・注意はパッケージごとの README.md に移した。そのパッケージを触る前に読む。
C++ の規約 (Google スタイル) とハードウェア構成は `tr_dokusute/README.md` にある。

- `tr_dokusute/README.md`
- `br_swervedrive/README.md`
- `nr27_sensors/README.md` (LiDAR)
- `nr27_localization/README.md` (自己位置)

## コーディング規約

- ROSに依存しない計算 (運動学など) は別ファイルに分けて gtest を書く
  (例: `swerve_kinematics.hpp` と `test/test_swerve_kinematics.cpp`)

### Python (launch ファイル)
ROS 2 の ament_flake8 / ament_pep257 に従う (シングルクォート、1行99文字、
docstring は1行目を空けて書き、要約は `.` で終える)。

### コメント
- コメントは日本語。何をしているかより「なぜそうしたか」を書く
  (過去の不具合・実測値・日付を残す)
- 物理量には単位を書く (`[m/s]`, `[rad, 出力軸]` など)

## 壊しやすいポイント

- **同じ `serial_tx_<ID>` に publish するノードは1つだけにする。** メッセージは24スロット丸ごとなので、
  複数ノードが送ると互いの担当外スロットを0で上書きする (1ユニット試験では、ステアと駆動でIDが別なので問題ない)
- **終了時の停止指令は pre-shutdown コールバックで送る。** `rclcpp::on_shutdown` や spin の後の publish は
  通信が切れた後なので届かない (damiao_ctrl で2026-10-14に実機確認)
- DMモータは全ゼロ (= DISABLE) が安全な既定値。`tr_steer_dm2325` は故障をラッチし、
  ノードを再起動するまで DISABLE のままにする (故障のまま有効化を繰り返さないため)
- 実機のモータを動かすコードなので、スロット割り当て・スケール・上限 (duty_max, 速度上限)・
  タイムアウト・停止処理に関わる変更は、意図しない動作につながりうることをユーザーに明示する
- 実機の値が決まっていないパラメータは `config/tr_dokusute.yaml` で【仮置き】と書いてある。
  勝手に確定値として扱わない
