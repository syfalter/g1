#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u

run_id="$(date +%Y%m%d_%H%M%S)"
raw_path="/home/syf/g1_upper_body_ws/calibration/right_arm_drag_refit_${run_id}.csv"
fit_path="/home/syf/g1_upper_body_ws/calibration/right_arm_payload_refit_${run_id}.yaml"

echo "Interactive controls:"
echo "WARNING: use this script only when the physical payload is unchanged."
echo "For a newly installed camera, use run_right_arm_camera_recalibrate.sh."
echo "  RECORD or 1  capture the current pose for 1.5 s"
echo "  NEXT   or 2  resume drag and move to the next pose"
echo "  EXIT   or 4  refit, restore gains, and exit"
echo "  ABORT        controlled abort without refitting"
echo "Raw log:       ${raw_path}"
echo "Selected cases: ${raw_path}.cases.csv"
echo "Temperature:    ${raw_path}.temperature.csv"
echo "Refitted model: ${fit_path}"

exec /home/syf/unitree_sdk2/build/bin/g1_right_arm_payload_calibration \
  "$1" --drag-refit I_UNDERSTAND_12_POSE_CALIBRATION \
  /home/syf/g1_upper_body_ws/calibration/right_arm_payload_fit_20260824_132917.yaml \
  /home/syf/g1_upper_body_ws/calibration/right_arm_payload_run_20260824_132917.csv \
  "${raw_path}" \
  "${fit_path}"
