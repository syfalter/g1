#!/usr/bin/env bash
# 在安全退出位置关闭整套应用（含RViz）；先渐进释放控制，再结束所有应用进程。
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --offline ]]; then
  shift
  exec bash "$root/scripts/task_editor_offline.sh" stop "$@"
fi
source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
source "$root/install/setup.bash"
exec 9>"/tmp/g1-task-panel-start-${UID}.lock"
flock -n 9 || { printf '%s\n' 'Another panel start/stop is running.' >&2; exit 2; }
exec /usr/bin/python3 "$root/scripts/task_panel_lifecycle.py" close "$@"
