#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
set -u

exec /home/syf/unitree_sdk2/build/bin/g1_right_arm_payload_calibration \
  "$1" --resume I_UNDERSTAND_12_POSE_CALIBRATION \
  /home/syf/g1_upper_body_ws/calibration/right_arm_candidate_poses.yaml \
  /home/syf/g1_upper_body_ws/calibration/right_arm_payload_run_20260824_132917.csv \
  /home/syf/g1_upper_body_ws/calibration/right_arm_payload_fit_20260824_132917.yaml \
  /home/syf/g1_upper_body_ws/calibration/right_arm_resume_to_pose8.yaml 8
