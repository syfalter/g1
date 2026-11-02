#!/usr/bin/env bash
set -eo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE" >&2
  exit 1
fi
echo "LEFT endpoint commissioning: finish the left admittance check first."
exec /home/syf/g1_upper_body_ws/scripts/run_g1_endpoint_motion_server.sh "$1" --rviz --arm left
