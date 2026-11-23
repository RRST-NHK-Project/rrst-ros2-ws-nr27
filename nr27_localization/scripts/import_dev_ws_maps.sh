#!/usr/bin/env bash
# dev_ws のシムの world (V1.1 準拠) から、AMCL 用の地図を作って maps/ へ取り込む。
#
#   nr27_localization/scripts/import_dev_ws_maps.sh            # LiDAR は床から 0.185 m (シムと同じ)
#   nr27_localization/scripts/import_dev_ws_maps.sh 0.20       # 実機の取り付け高さが違うとき
#
# フィールドの寸法の正本は dev_ws の make_field.py (world は手で編集しない生成物)。
# 地図を作るのも dev_ws の sdf_to_map.py で、引数は dev_ws/docs/field.md の「層ごとの地図」と同じ。
# 高さ 0.185 m なら dev_ws の maps/field_2027_{ground,l1} と1マスも違わない地図になる。
# TR が動けるのはグラウンドとトランスファー (L1 と同じ高さの踊り場) まで (ルールブック【3.2.1】) なので、
# L2 の地図は作らない (2026-11-23)。
# world が変わったら (dev_ws で make_field.py を流し直したら) これを流し直してコミットする。
set -euo pipefail

DEV_WS=${DEV_WS:-$HOME/dev_ws}
H=${1:-0.185}   # 床から LiDAR の走査面までの高さ [m]
HERE=$(cd "$(dirname "$0")/.." && pwd)
OUT=$HERE/maps
WORLD=$DEV_WS/src/robocon2027_sim/worlds/field_2027.sdf
TOOL=$DEV_WS/src/robocon2027_sim/scripts/sdf_to_map.py

for f in "$WORLD" "$TOOL"; do
  [[ -f $f ]] || { echo "見つからない: $f (DEV_WS=$DEV_WS)" >&2; exit 1; }
done

# 床の高さ: グラウンド 0、L1 (トランスファー) 0.6 [m]
z() { python3 -c "print(round($1 + $H, 4))"; }
M=(python3 "$TOOL" --world "$WORLD" --out "$OUT" --grid amcl --mirror --clip-edge unknown)
# L1 は 6x6 と両チームのトランスファー (1x1) を床として扱う
CL=(--clip -3.0 -3.0 3.0 3.0 --clip -4.0 -1.1 -3.0 -0.1 --clip 3.0 -1.1 4.0 -0.1)

"${M[@]}" --z "$(z 0.0)" --clip -5.5 -5.5 5.5 5.5 --seed -4.0 -4.0 --name field_2027_ground
"${M[@]}" --z "$(z 0.6)" "${CL[@]}" --seed -2.2 -2.2 --name field_2027_l1

COMMIT=$(git -C "$DEV_WS" log -1 --format='%h %ad %s' --date=short 2>/dev/null || echo '不明')
DIRTY=$(git -C "$DEV_WS" status --porcelain -- src/robocon2027_sim/worlds/field_2027.sdf 2>/dev/null)
cat > "$OUT/SOURCE.md" <<EOF
# 地図の出どころ

\`scripts/import_dev_ws_maps.sh\` が作った。手で編集しない。

- world: dev_ws \`src/robocon2027_sim/worlds/field_2027.sdf\`${DIRTY:+ (未コミットの変更あり)}
- dev_ws のコミット: $COMMIT
- 走査面: 床から $H m (グラウンド $(z 0.0) / L1 $(z 0.6) m)
EOF
echo "取り込んだ: $OUT (走査面 床から $H m)"
