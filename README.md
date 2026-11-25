# **rrst-ros2-ws-nr27**

## 1. 概要

立命館大学ロボット技術研究会（RRST）NHKプロジェクトの、NHK学生ロボコン2027（ABU Robocon 2027）向け ROS 2 ワークスペースです。TR・BR の制御パッケージおよびセンサ・自己位置推定の設定を管理しています。

| ブランチ | 説明 | ビルド状況 |
|:---|:---|:---|
| **main** | 安定版ブランチ（動作確認済み） | [![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws-nr27/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/rrst-ros2-ws-nr27/actions/workflows/main_jazzy_build_and_test.yml) |
| **develop** | 開発ブランチ（作業中） | [![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws-nr27/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=develop&event=push)](https://github.com/RRST-NHK-Project/rrst-ros2-ws-nr27/actions/workflows/main_jazzy_build_and_test.yml) |

---

## 2. 動作環境

| 項目 | 内容 |
|:---|:---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| メモリ | 16 GB 以上推奨 |

> **注意**: ビルド中にフリーズする場合はメモリ不足が疑われます。スワップ領域を追加するか、`--parallel-workers` でスレッド数を制限してください。

---

## 3. セットアップとビルド

このリポジトリはルートディレクトリ自体を `colcon` ワークスペースとして扱います。

### 3.1 リポジトリのクローン

**main（安定版）**

```bash
git clone https://github.com/RRST-NHK-Project/rrst-ros2-ws-nr27.git
cd rrst-ros2-ws-nr27
```

**develop（開発中）**

```bash
git clone -b develop https://github.com/RRST-NHK-Project/rrst-ros2-ws-nr27.git
cd rrst-ros2-ws-nr27
```

### 3.2 依存パッケージの導入とビルド

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

スレッド数を制限してビルドする場合:

```bash
colcon build --symlink-install --parallel-workers 4
```

特定のパッケージのみビルド・テストする場合:

```bash
colcon build --symlink-install --packages-up-to <パッケージ名>
colcon test --packages-select <パッケージ名>
colcon test-result --verbose --test-result-base build/<パッケージ名>
```

> **実機・シムで実行するとき**: ビルドは ROS 2 Jazzy と `rosdep` の依存のみで通りますが、実行時は通信ノードやドライバ（`ros2can`、`rrst-ros2-ws` の `damiao_ctrl`、各 LiDAR ドライバ）が入ったワークスペースを先に `source` してください。手順は各パッケージの README に記載しています。

---

## 4. パッケージ構成

| ディレクトリ | 機体 | 説明 | 詳細 |
|:---|:---|:---|:---|
| `tr_dokusute` | TR | 独立ステア足回り制御（運動学・PID・DM2325 操舵・マブチ735 駆動） | [README](tr_dokusute/README.md) |
| `br_swervedrive` | BR | 独ステ1ユニット（DM2325 操舵 + DM3519 駆動）の試験制御ノード | [README](br_swervedrive/README.md) |
| `nr27_sensors` | TR | LiDAR 3機種（UST-30LX / TG30 / LD19）の launch・設定 | [README](nr27_sensors/README.md) |
| `nr27_localization` | TR | LiDAR + AMCL による自己位置推定とフィールド地図 | [README](nr27_localization/README.md) |
| `common` | 共通 | 共通ヘッダライブラリ（PID 制御・パケット操作、[rrst-ros2-ws](https://github.com/RRST-NHK-Project/rrst-ros2-ws) からの写し） | — |

---

## 5. AI で開発するとき

共通の規則は [AGENTS.md](AGENTS.md) にまとめています。Codex はサブフォルダで起動しても `AGENTS.md` を直接読み込みます。Claude Code は `CLAUDE.md` 内の `@AGENTS.md` 経由で読み込みます。

- Claude Code はリポジトリのルートで起動してください。
- サブフォルダで起動すると、`@AGENTS.md` が作業フォルダ外を指す import になるため、初回に読み込みの許可を求められます。ここで拒否すると以後は読み込まれず、再確認も出ません。
- `claude -p` などの非対話実行では確認が出ず、サブフォルダからは読み込まれません。必ずルートで実行してください。
- 読み込まれたかどうかは `/context` の Memory files に `AGENTS.md` が表示されているかで確認できます。

---

## 6. ライセンス

このリポジトリは [MIT License](LICENSE) のもとで公開されています。

---

## 7. クレジット

Developed by NHK Project, RRST, Ritsumeikan University, Japan (2026)
- 公式サイト: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)
