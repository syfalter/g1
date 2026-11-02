#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u

run_id="$(date +%Y%m%d_%H%M%S)"
raw_path="/home/syf/g1_upper_body_ws/calibration/right_arm_camera_recalibrate_${run_id}.csv"
fit_path="/home/syf/g1_upper_body_ws/calibration/right_arm_camera_fit_${run_id}.yaml"

echo "NEW-PAYLOAD recalibration: old fit is used only for active support."
echo "Only cases recorded in this run enter the new fit; at least 6 are required."
echo "Recommended: record 8 diverse static poses after installing the camera."
echo "Controls: RECORD/1, NEXT/2, EXIT/4, ABORT"
echo "Raw log:        ${raw_path}"
echo "Selected cases: ${raw_path}.cases.csv"
echo "Temperature:    ${raw_path}.temperature.csv"
echo "New fit:        ${fit_path}"

exec /home/syf/unitree_sdk2/build/bin/g1_right_arm_payload_calibration \
  "$1" --drag-recalibrate I_UNDERSTAND_12_POSE_CALIBRATION \
  /home/syf/g1_upper_body_ws/calibration/right_arm_payload_fit_20260824_132917.yaml \
  "${raw_path}" \
  "${fit_path}"
