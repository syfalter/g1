#!/usr/bin/env bash
# 一起启动新面板、控制器、规划服务和RViz；保留准备位，重新确认R2+A，不自动ARM。
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --offline ]]; then
  shift
  exec bash "$root/scripts/task_editor_offline.sh" start "$@"
fi
source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
source "$root/install/setup.bash"
exec 9>"/tmp/g1-task-panel-start-${UID}.lock"
flock -n 9 || { printf '%s\n' 'Panel startup is already running.' >&2; exit 2; }
if [[ $# -gt 0 && "$1" != --* ]]; then
  config="$1"
  shift
  set -- --planning-config "$config" "$@"
fi
exec /usr/bin/python3 "$root/scripts/task_panel_lifecycle.py" start "$@"
