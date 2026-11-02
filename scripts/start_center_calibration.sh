#!/usr/bin/env bash
# 独立测量面板：不启动 DDS、控制器、RViz，不发送机器人命令。
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec /usr/bin/python3 "$root/scripts/center_calibration_panel.py" "$@"
