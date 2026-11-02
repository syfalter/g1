#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u

runner=/home/syf/unitree_sdk2/build/bin/g1_right_arm_payload_calibration
candidates=/home/syf/g1_upper_body_ws/calibration/right_arm_candidate_poses.yaml
run_id=$(date +%Y%m%d_%H%M%S)
output_csv=/home/syf/g1_upper_body_ws/calibration/right_arm_payload_run_${run_id}.csv
output_fit=/home/syf/g1_upper_body_ws/calibration/right_arm_payload_fit_${run_id}.yaml

echo "Candidates: ${candidates}"
echo "Samples:    ${output_csv}"
echo "Fit:        ${output_fit}"

exec "${runner}" "$1" --execute I_UNDERSTAND_12_POSE_CALIBRATION \
  "${candidates}" "${output_csv}" "${output_fit}"
