#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode=offline
selected=
build=false
args=()
need_value() { if (($# < 2)) || [[ -z "$2" || "$2" == --* ]]; then printf '%s 需要参数\n' "$1" >&2; exit 2; fi; }
while (($#)); do
  case "$1" in
    --offline|--live)
      if [[ -n "$selected" && "$selected" != "$1" ]]; then printf '不能同时指定在线和离线\n' >&2; exit 2; fi
      selected=$1; mode=${1#--}; shift ;;
    --session) need_value "$@"; args+=(session_file:="$2"); shift 2 ;;
    --network) need_value "$@"; args+=(network:="$2"); shift 2 ;;
    --ssh) need_value "$@"; args+=(ssh_host:="$2"); shift 2 ;;
    --peer) need_value "$@"; args+=(peer:="$2"); shift 2 ;;
    --rviz) args+=(use_rviz:=true); shift ;;
    --backend-only) args+=(use_window:=false); shift ;;
    --build) build=true; shift ;;
    -h|--help)
      printf '%s\n' '实验 C：默认离线，不启动或停止机器人服务。' \
        "用法: bash $0 [--offline | --live] [--session JSON] [--rviz] [--build] [--backend-only]" \
        '可选: --network IFACE --ssh USER@HOST --peer IP' \
        '导纳停稳采样：新流取帧与全窗口实际姿态检查通过后可求解；不自动应用候选。'
      exit 0 ;;
    *) printf '未知参数：%s\n' "$1" >&2; exit 2 ;;
  esac
done
set +u
if [[ "$mode" == offline ]]; then
  source /opt/ros/humble/setup.bash
  source /home/syf/unitree_ws/unitree_ros2/install/setup.bash
else
  source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
fi
if $build || [[ ! -x "$root/install/g1_tcp_diagnostics/lib/g1_tcp_diagnostics/camera_c_window" ]] || [[ ! -x "$root/install/g1_tcp_diagnostics/lib/g1_tcp_diagnostics/camera_c_actual_reader" ]]; then
  (cd "$root" && colcon build --packages-select g1_tcp_diagnostics --symlink-install)
fi
source "$root/install/setup.bash"
if [[ "$mode" == offline ]]; then source "$root/scripts/tcp_a_offline_environment.sh"; fi
set -u
printf 'C 模式=%s；不 ARM、不改手型、不启停 owner。关闭只退出 C。\n' "$mode"
exec ros2 launch g1_tcp_diagnostics experiment_c.launch.py mode:="$mode" "${args[@]}"
