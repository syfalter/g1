#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/syf/g1_upper_body_ws/install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/g1-workbench-application-${UID}.lock"
if ! flock -n 9; then
  printf '%s\n' 'Offline workbench application is already running.' >&2
  exit 1
fi
exec ros2 launch /home/syf/g1_upper_body_ws/scripts/workbench_application.launch.py "$@"
