#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u

run_id="$(date +%Y%m%d_%H%M%S)"
output_path="/home/syf/g1_upper_body_ws/calibration/right_arm_admittance_${run_id}.csv"

echo "Seven-axis right-arm sensorless admittance test"
echo "Output log: ${output_path}"
echo "After the final ENTER, hold physical F3 (DDS 0x0080) to drag."
echo "Release F3 to brake/hold; STOP restores normal gains."

exec /home/syf/unitree_sdk2/build/bin/g1_right_arm_payload_calibration \
  "$1" --full-arm-admittance I_UNDERSTAND_12_POSE_CALIBRATION \
  /home/syf/g1_upper_body_ws/calibration/right_arm_camera_fit_20260824_185416.yaml \
  "${output_path}"
