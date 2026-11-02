#!/usr/bin/env bash
set -euo pipefail
root=/home/syf/g1_upper_body_ws
offline=false
use_rviz=true
session_file=''
robot_urdf=''
while (($#)); do
  case "$1" in
    --offline) offline=true; shift ;;
    --backend-only) use_rviz=false; shift ;;
    --session) session_file="$2"; shift 2 ;;
    --robot-urdf) robot_urdf="$2"; shift 2 ;;
    *) printf '用法: bash scripts/start_tcp_a.sh [--offline] [--backend-only] [--session FILE] [--robot-urdf FILE]\n' >&2; exit 2 ;;
  esac
done
set +u
if [[ "$offline" == true ]]; then
  source /opt/ros/humble/setup.bash
  source /home/syf/unitree_ws/unitree_ros2/install/setup.bash
  source "$root/install/setup.bash"
  source "$root/scripts/tcp_a_offline_environment.sh"
else
  source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
  source "$root/install/setup.bash"
fi
set -u
printf '实验A：采样后端只读；布置页的真机按钮须单独确认并交给原任务进程。启动不ARM、不伸指、不切模式、不启动控制器。\n'
launch_args=(offline:="$offline" use_rviz:="$use_rviz")
if [[ -n "$session_file" ]]; then launch_args+=(session_file:="$session_file"); fi
if [[ -n "$robot_urdf" ]]; then launch_args+=(robot_urdf:="$robot_urdf"); fi
exec ros2 launch g1_tcp_diagnostics experiment_a.launch.py \
  "${launch_args[@]}"
