#!/usr/bin/env bash
set -eo pipefail
if [[ $# -lt 1 || $# -gt 2 || ! "$1" =~ ^[A-Za-z0-9_-]{1,64}$ ]]; then
  echo "Usage: $0 POSE_LABEL [NETWORK_INTERFACE=eth0]" >&2
  exit 1
fi
label="$1"
network="${2:-eth0}"
ip link show dev "$network" >/dev/null
source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${network}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
root="/home/syf/g1_upper_body_ws"
output="$root/logs/taught_poses/${label}_$(date +%Y%m%d_%H%M%S_%N)"
echo "Read-only stationary joint capture; no motor or hand commands."
/usr/bin/python3 "$root/scripts/capture_workbench_start_pose.py" --output "$output" --label "$label"
exec /usr/bin/python3 "$root/scripts/record_workbench_pose.py" --capture "$output" --label "$label"
