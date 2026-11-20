# rrst-ros2-ws-nr27
NHKロボコン2027年度用

|  | Description | Build Status |
|---|---|---|
| **main** | Stable branch (verified) | [![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws-nr27/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/rrst-ros2-ws-nr27/actions/workflows/main_jazzy_build_and_test.yml) |
---

## AI で開発するとき

共通の決まりは [AGENTS.md](AGENTS.md) にある。Codex は `AGENTS.md` を直接読む (サブフォルダで起動しても読む)。Claude Code は `CLAUDE.md` の `@AGENTS.md` から読む。

- Claude Code はリポジトリのルートで起動する
- サブフォルダで起動すると、`@AGENTS.md` が作業フォルダの外を指す import になり、最初に読み込んでよいかを聞かれる。許可する。拒否すると以後は読み込まれず、確認も出ない
- `claude -p` など対話でない実行では確認が出ず、サブフォルダからは読み込まれない。ルートで実行する
- 読み込まれたかは `/context` の Memory files で確かめる。`AGENTS.md` が出ていればよい
