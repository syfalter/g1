#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
capture="${1:-$root/logs/vision_task_seed_20260908_001/vision_capture.json}"
intrinsics="${2:-/home/syf/unitree_ws/head_camera_intrinsics.json}"
pose="${3:-$root/assets/workbench/task_start_pose.json}"
arm="${4:-left}"
rgbd_dir="${5:-$(dirname "$capture")}"
rgbd_args=()
if [[ -f "$rgbd_dir/manifest.json" ]]; then
  rgbd_args=(--rgbd-dir "$rgbd_dir")
elif [[ $# -ge 5 ]]; then
  printf '%s\n' "RGB-D archive incomplete: $rgbd_dir/manifest.json missing" >&2
  exit 1
fi
if [[ "$arm" != left && "$arm" != right && "$arm" != both ]]; then
  printf '%s\n' 'ARM must be left, right, or both.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=92
export ROS_LOCALHOST_ONLY=1
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/g1-vision-task-preview-${UID}.lock"
if ! flock -n 9; then
  printf '%s\n' 'Vision task preview is already running.' >&2
  exit 1
fi
tmpdir="$(mktemp -d --tmpdir g1-vision-task-preview-XXXXXX)"
artifact="$tmpdir/preview.json"
cleanup() {
  trap - EXIT INT TERM
  jobs -pr | xargs -r kill 2>/dev/null || true
  rm -f "$artifact"
  rmdir "$tmpdir" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
/usr/bin/python3 "$root/scripts/vision_task_geometry.py" "$capture" \
  --intrinsics "$intrinsics" --output "$artifact" \
  --entry-key "${G1_VISION_ENTRY_KEY:-operation_diagonal_intersection}" \
  --radial-offset-m "${G1_VISION_RADIAL_OFFSET_M:-0}" \
  --height-offset-m "${G1_VISION_HEIGHT_OFFSET_M:-0}"
/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher --ros-args \
  -r __node:=camera_axis_robot_state \
  --params-file "$root/assets/camera_reference/offline_robot_params.yaml" &
/usr/bin/python3 "$root/scripts/vision_task_preview.py" "$artifact" \
  --offline-pose "$pose" --arm "$arm" "${rgbd_args[@]}" &
rviz2 -d "$root/assets/camera_reference/vision_task.rviz" &
rviz_pid=$!
wait "$rviz_pid"
