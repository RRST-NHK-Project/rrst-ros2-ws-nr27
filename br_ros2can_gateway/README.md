# br_ros2can_gateway

BR の自律制御ソフトと `ros2can` の間に 1 つだけ配置する接続パッケージ（SW-65・SW-66・SW-71、dev_ws の決定事項 58・60〜62・67）。
上位からの指令を `ros2can` の 24 スロット形式へ変換し、単一送信元・途絶/故障検知・非常停止・スイッチ入力を統括する。
足回りの式（`swerve.py`）と値（`config/swerve.yaml`）もここに置く。このパッケージは dev_ws に頼らず、dev_ws のシム（`robocon2027_sim`）の方がこれに頼る。
BR の足回りのモータは、このパッケージがスロットを直に書き、`damiao_ctrl` を通さない（2026-11-27）。`br_swervedrive` は同じ `serial_tx_201` に書くので、同時に上げない。

経緯と実測値は dev_ws（非公開）の `docs/control.md` および [`ros2can` Issue #4](https://github.com/RRST-NHK-Project/ros2can/issues/4) を参照。

---

## 1. アーキテクチャとファイル構成

```text
追従器 / Nav2 ──[/cmd_vel]──▶ swerve_drive.py（逆運動学）
                                 │
                          [/drive/module_cmd]
                                 ▼
                       gateway_node.py（仲介ノード）
                        ├─ /serial_tx_201・204 ──▶ ros2can / ros2can_fake.py ──▶ Damiao ESC
                        ├─ /serial_rx_201・204 ◀── 帰還（200 Hz）＋ 多回転展開・暴走監視
                        └─ /serial_rx_101      ◀── 入力用 ESP32（スタート / 陣営 / 非常停止 / フリーズ）
```

| ファイル | 役割 |
|---|---|
| [`config/gateway_map.yaml`](config/gateway_map.yaml) | デバイス ID・スロット割当・ギア比・安全しきい値・入力ピン定義の唯一の設定表 |
| [`config/swerve.yaml`](config/swerve.yaml) | 4輪独立ステアの値（モジュールの位置・周速とステアの上限・ステアの範囲） |
| [`scripts/swerve.py`](scripts/swerve.py) | 4輪独立ステアの運動学・逆運動学・制約（ROS 非依存の純関数。dev_ws の軌道生成・模型・追従器も同じ関数を使う） |
| [`scripts/gateway_map.py`](scripts/gateway_map.py) | 割り当て表の読込・単位換算・25 rad 多回転展開・状態機械（ROS 非依存の純関数群） |
| [`scripts/swerve_drive.py`](scripts/swerve_drive.py) | `/cmd_vel` と現在ステア角から4輪目標角・周速を計算する逆運動学ノード |
| [`scripts/gateway_node.py`](scripts/gateway_node.py) | 50 Hz 周期送信・他送信元監視・安全停止・入力スイッチ読みを担う仲介ノード |
| [`scripts/ros2can_fake.py`](scripts/ros2can_fake.py) | `MODE_DAMIAO`・入力用 ESP32 の模擬ノード（シム・単体結合試験専用） |
| [`scripts/verify_gateway_map.py`](scripts/verify_gateway_map.py) | `gateway_map.py` 単体試験（16 項目・ROS 不要） |
| [`scripts/verify_swerve.py`](scripts/verify_swerve.py) | `swerve.py` 単体試験（ROS 不要。CasADi があれば記号の式も確かめる） |
| [`scripts/verify_gateway_node.py`](scripts/verify_gateway_node.py) | 3 ノード結合＋異常注入試験（25 項目・Gazebo 不要） |

Gazebo を通す試験（H1 フリーズ・H2 途絶/非常停止・H3 スタートスイッチ、25 項目）は、作戦の層とシムが要るので dev_ws の `robocon2027_sim/scripts/verify_gateway_sim.py` にある。

---

## 2. モータ割当・単位換算・原点

- **ステア（`device_id: 201`, `MODE_POS = 2`）**: DM2325（減速比 `13:49`）、スロット 0〜3 が左前(0)・左後(1)・右後(2)・右前(3)。角度範囲は **±135°**（モータ軸 **±8.88 rad**、dev_ws の決定事項62）、速度上限 `30 rad/s`。
- **駆動（`device_id: 204`, `MODE_VEL = 1`）**: DM3519（ギア比 `23:33`）、車輪直径 `80 mm`（半径 `0.04 m`）。出力軸上限 `45.6 rad/s` に対して `swerve.yaml` の周速上限は `1.25 m/s`（`23/33 × 0.04 × 45.6 ≈ 1.271 m/s` 以下）。
- **位置展開と原点**: `serial_rx_[ID]` の位置値を幅 `25.0 rad`（±12.5 rad）で `unwrap()` 展開する（`ros2can` 側の 32768 幅展開トピックは使用しない）。電源投入時が 0 のため、原点オフセット `zeros` を引数または原点取得ノードから受け取る。
- **オドメトリ禁止事項**: DM3519 はロータ側（19.2 倍）基準で低速時の位置・速度誤差が出るため、オドメトリには使わず測定輪・外付けエンコーダを別途用いる。

---

## 3. 安全設計と入力仕様

| 項目 | 仕様 |
|---|---|
| 単一送信元強制 | `/serial_tx_<dev>` に他ノードの publisher が見つかれば起動拒否または即脱力（`other_tx`） |
| 起動・再稼働 | 起動直後は脱力（全スロット 0）。`~/arm` 要求（またはスタートスイッチ、`--auto-arm`）で許可 |
| 脱力ラッチ条件 | 上流途絶・帰還途絶（`0.2 s` 仮）・非常停止・モータ故障(15)・暴走検知（`0.3 s` 継続）で脱力し、原因解消後も次回の `~/arm` まで解除しない |
| フリーズ動作 | 駆動を速度 `0`、ステアを現在角で保持（`encode_hold`、モータ有効のまま）。解除で復帰 |
| 終了時安全処理 | `SIGINT` / `SIGTERM` 時に脱力フレームを 5 回送信してから `rclpy` を終了する |
| 入力 ESP32 途絶時 | `input_rules.lost: hold`（dev_ws の決定事項67）。帰還が途切れたら速度 0 で保持し、復帰で走行再開 |
| 実機ガード | `gateway_map.yaml` の `provisional` が空になるまで `--hw real` は起動しない |

> **注意**:
> `provisional` に残っている未確定項目（2026-11-27 時点）：`devices.inputs`、`motors.sign`、`motors.home`、`inputs`、`input_rules`、`runaway`、`cmd_timeout`、`fb_timeout`。実機決定ごとに同ファイルを更新し、`provisional` から外すこと。

---

## 4. 検証と実行コマンド

```bash
# 割り当て表と仮置き項目の一覧
python3 br_ros2can_gateway/scripts/gateway_map.py

# 単体・結合試験（Gazebo 不要）
python3 br_ros2can_gateway/scripts/verify_gateway_map.py       # 16 項目・1 秒
python3 br_ros2can_gateway/scripts/verify_swerve.py            # 足回りの式
ros2 run br_ros2can_gateway verify_gateway_node.py                 # 25 項目・約 1〜2 分

# Gazebo 統合・実機構成（模擬）試験（dev_ws を同じワークスペースに置いたとき）
ros2 run robocon2027_sim verify_gateway_sim.py                     # H1〜H3（25 項目・約 7〜10 分）
ros2 launch robocon2027_sim hw_bringup.launch.py hw:=fake          # 実機構成一式（模擬スイッチ・模擬世界）
ros2 launch robocon2027_sim mission.launch.py sim:=true bringup:=true hw:=fake
```

> **注意**:
> 模擬の `ros2can_fake.py` は通信途絶時のラッチや簡易回転を再現するが、脱力時のランプ滑落、モータトルク飽和、CAN 欠損は再現しない（dev_ws（非公開）の `docs/control.md` 参照）。
