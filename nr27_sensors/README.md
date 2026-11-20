# LiDAR (`nr27_sensors`)

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
