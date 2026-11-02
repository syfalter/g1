#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
window=true
rviz=false
mode=offline
selected_mode=
network=eth0
host=unitree@192.168.0.249
peer=192.168.123.164
check=false
build=false
replace=false
args=()
usage() {
  printf '%s\n' '实验B：RGB/深度一致性，只读相机，不控制机器人。' \
    "用法: bash $0 [--live | --offline] [选项]" \
    '  --live                 连接原vision_service；启动仅检查，不自动采样' \
    '  --offline              离线回放（默认）' \
    '  --network IFACE        相机RPC网卡，默认eth0' \
    '  --ssh USER@HOST        图像下载地址，默认unitree@192.168.0.249' \
    '  --peer IP              DDS对端，默认192.168.123.164' \
    '  --check                仅在线连接检查，不开窗口、不采图' \
    '  --replace              正常关闭旧B后端/窗口，不触碰任务/A控制器' \
    '  --build                先构建g1_tcp_diagnostics包' \
    '  --session JSON         加载会话快照' \
    '  --import PATH          导入历史RGB或RGB-D目录' \
    '  --backend-only         不打开窗口' \
    '  --rviz                 在RViz中打开B面板' \
    '  -h, --help             显示帮助'
}
need_value() { if (($# < 2)) || [[ -z "$2" || "$2" == --* ]]; then printf '%s 需要参数\n' "$1" >&2; exit 2; fi; }
while (($#)); do
  case "$1" in
    --offline|--live)
      if [[ -n "$selected_mode" && "$selected_mode" != "$1" ]]; then printf '不能同时指定在线和离线模式\n' >&2; exit 2; fi
      selected_mode=$1; mode=${1#--}; shift ;;
    --network) need_value "$@"; network=$2; shift 2 ;;
    --ssh) need_value "$@"; host=$2; shift 2 ;;
    --peer) need_value "$@"; peer=$2; shift 2 ;;
    --check) check=true; shift ;;
    --build) build=true; shift ;;
    --replace) replace=true; shift ;;
    --backend-only) window=false; shift ;;
    --rviz) rviz=true; shift ;;
    --session) need_value "$@"; args+=(session_file:="$2"); shift 2 ;;
    --import) need_value "$@"; args+=(import_path:="$2"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf '未知选项: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done
if $check; then
  [[ "$mode" == live ]] || { printf '%s\n' '--check 必须配合 --live' >&2; exit 2; }
  exec /usr/bin/python3 "$root/scripts/camera_b_node.py" --live --check-only --network "$network" --ssh "$host" --peer "$peer"
fi
if $replace; then
  PYTHONPATH="$root/scripts${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -c 'from camera_b_node import stop_existing_backends; stop_existing_backends()'
fi
set +u
source /opt/ros/humble/setup.bash
if $build; then
  (cd "$root" && colcon build --packages-select g1_tcp_diagnostics --symlink-install)
fi
source "$root/install/setup.bash"
source "$root/scripts/tcp_a_offline_environment.sh"
set -u
printf '实验B模式=%s；界面localhost/domain96，仅在线RPC子进程访问相机；不发送运动命令。\n' "$mode"
exec ros2 launch g1_tcp_diagnostics experiment_b.launch.py mode:="$mode" network:="$network" ssh_host:="$host" peer:="$peer" use_window:="$window" use_rviz:="$rviz" "${args[@]}"
