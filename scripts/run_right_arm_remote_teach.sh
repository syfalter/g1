#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE" >&2
  exit 1
fi

exec 8>"/tmp/g1-arm-sdk-owner-${UID}.lock"
if ! flock -n 8; then
  echo "Another guarded arm controller owns the local lock; refusing to start teaching." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
set -u

run_id="$(date +%Y%m%d_%H%M%S)"
output_prefix="/home/syf/g1_upper_body_ws/calibration/right_arm_remote_teach_${run_id}.csv"

echo "Right-arm remote teaching daemon"
echo "Session prefix: ${output_prefix}"
echo "F1+X hold 0.8 s: start/stop the active hold controller"
echo "Physical F3 (DDS 0x0080): hold continuously to drag; release to brake/hold"
echo "F1+A press: capture joint snapshot"
echo "L2+B: native emergency damping and immediate silent latch"
echo "Camera capture interface is reserved but disabled"

exec /home/syf/unitree_sdk2/build/bin/g1_right_arm_payload_calibration \
  "$1" --remote-admittance I_UNDERSTAND_12_POSE_CALIBRATION \
  /home/syf/g1_upper_body_ws/calibration/right_arm_camera_fit_20260824_185416.yaml \
  "${output_prefix}"
