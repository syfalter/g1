#!/usr/bin/env bash
set -eo pipefail
mode="${1:?server or worker}"
run_directory="$(realpath "${2:?run directory}")"
shard="${3:?shard 0 through 7}"
shift 3
case "$shard" in [0-7]) ;; *) exit 2 ;; esac
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="$((78 + shard))"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
case "$mode" in
  server) exec ros2 launch "$root/scripts/workbench_robustness.launch.py" "run_directory:=$run_directory" ;;
  worker) exec /usr/bin/python3 "$root/scripts/workbench_robustness.py" run --root "$run_directory" --shard "$shard" "$@" ;;
  *) printf '%s\n' 'Expected server or worker' >&2; exit 2 ;;
esac
