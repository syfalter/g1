#!/usr/bin/env bash
set -eo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE" >&2
  exit 1
fi
exec 8>"/tmp/g1-arm-sdk-owner-${UID}.lock"
if ! flock -n 8; then
  echo "Another guarded arm controller owns the local lock; refusing left teaching." >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
binary=/home/syf/unitree_sdk2/build/bin/g1_left_arm_payload_calibration
fit=/home/syf/g1_upper_body_ws/calibration/left_arm_right_copy_seed_20260908.yaml
output="/home/syf/g1_upper_body_ws/calibration/left_arm_remote_teach_$(date +%Y%m%d_%H%M%S_%N).csv"
echo "LEFT ARM TEST: copied right-arm gains and payload seed; NOT left calibrated."
echo "Keep the base fixed and the arm clear of the table. Begin near the ready pose."
echo "F1+X hold 0.8 s: take control; wait for setup before touching the LEFT arm."
echo "Hold physical F3 to drag; release to brake/hold. F1+A records a left joint snapshot."
echo "Right arm remains supported at its captured position; no automatic arm movement."
echo "L2+B remains native emergency damping. Legacy admittance fault exits still apply."
echo "Log prefix: $output"
exec "$binary" "$1" --remote-admittance I_UNDERSTAND_12_POSE_CALIBRATION "$fit" "$output"
