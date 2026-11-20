# 自己位置 (`nr27_localization`)

LiDAR + AMCL (nav2_amcl)。地図は走って作らず、**dev_ws (RRST-NHK-Project/dev_ws、BR のシム環境) の world から作ったものを取り込む。**
フィールドの寸法の正本は dev_ws の `make_field.py` (ルールブック V1.1・付録の図面に合わせてある)。nr27 に寸法を書き写さない。

```bash
# 実機 (UST-30LX)
ros2 launch nr27_localization localization.launch.py x:=-4.1 y:=-4.8 yaw:=1.5708
# dev_ws のシムとつなぐ (先に別端末で: ros2 launch robocon2027_sim sim.launch.py world:=field_2027 drive:=omni
#   x:=-4.1 y:=-4.8 yaw:=1.5708。dev_ws と nr27 の両方の install を source する)
ros2 launch nr27_localization localization.launch.py sim:=true x:=-4.1 y:=-4.8 yaw:=1.5708
# world が変わったら地図を取り込み直してコミットする (引数は床から LiDAR までの高さ、既定 0.185 m。
#   dev_ws の場所は環境変数 DEV_WS で渡す、既定 ~/dev_ws)
nr27_localization/scripts/import_dev_ws_maps.sh
```

- 地図は `maps/field_2027_{ground,l1,l2}`。dev_ws の `sdf_to_map.py` を dev_ws/docs/field.md と同じ引数で流す。
  高さ 0.185 m なら dev_ws の地図と1マスも違わない (2026-11-18 に確認)。出どころは `maps/SOURCE.md`
- 座標は dev_ws と同じ: 原点はフィールド中央、赤が西 (-x)、北が +y。赤のスタートゾーンは (-4.1, -4.8) で北向き
- `sim:=true` の違いは、スキャンが `/scan`、`use_sim_time`、odom -> base_link を Gazebo が出すこと。
  **dev_ws の `localization.launch.py` と同時に上げない** (map -> odom が2か所から出る)
- AMCL の値は dev_ws の `config/localization.yaml` (シムで確かめた値) と同じで、運動モデルだけ Omni
- 階 (level) ごとに地図が違う。走りながらの切り替えはまだ無い。dev_ws は `switch_layer.py` で地図の差し替えと
  `/initialpose` を行い、**L2 では AMCL を止める** (L2 で LiDAR に写るのはセントラルピラーだけ)
- **LiDAR に写らない柵が多い。** V1.1 の付録で L1 外周バリアとセンターディバイダーは 50 mm、境界は 100 mm で、
  床から 0.185 m の LiDAR には写らない。L1 で写るのは L2 の台、グラウンドでは L1 の側面・ランプ・階段・ピラーだけ。
  dev_ws はトランスファーやスポットの手前で深度カメラで y を直している (dev_ws/docs/localization.md)
- シムでの確認 (2026-11-18): スタートから北・東へ走り 180° 回るまで誤差 0.02 m 以内。L1 の南西の角の外 (-3.3, -3.6) で
  西を向くと x が 0.11〜0.18 m ずれた。dev_ws 自身の AMCL でも同じ所で 0.11 m で、地形 (見えるのが角だけ) によるもの
- 実機は odom -> base_link (オドメトリ) がまだ無い。`fake_odom:=true` は止まって重なりを見る用で、動くと追従しない
