#!/usr/bin/env bash

# ROS 2 Humble + Unitree G1 real-robot environment.
# Source this file from a fresh shell:
#   source ~/unitree_ws/env/setup_unitree_humble_g1.sh

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This script must be sourced, not executed." >&2
  exit 1
fi

UNITREE_G1_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITREE_G1_WS_ROOT="$(cd "${UNITREE_G1_ENV_DIR}/.." && pwd)"
UNITREE_G1_ROS2_ROOT="${UNITREE_G1_WS_ROOT}/unitree_ros2"

UNITREE_G1_ROS_SETUP="/opt/ros/humble/setup.bash"
UNITREE_G1_DDS_SETUP="${UNITREE_G1_ROS2_ROOT}/cyclonedds_ws/install/setup.bash"
UNITREE_G1_EXAMPLE_SETUP="${UNITREE_G1_ROS2_ROOT}/install/setup.bash"

for UNITREE_G1_SETUP_FILE in \
  "${UNITREE_G1_ROS_SETUP}" \
  "${UNITREE_G1_DDS_SETUP}" \
  "${UNITREE_G1_EXAMPLE_SETUP}"; do
  if [[ ! -f "${UNITREE_G1_SETUP_FILE}" ]]; then
    echo "Missing required setup file: ${UNITREE_G1_SETUP_FILE}" >&2
    unset UNITREE_G1_ENV_DIR UNITREE_G1_WS_ROOT UNITREE_G1_ROS2_ROOT
    unset UNITREE_G1_ROS_SETUP UNITREE_G1_DDS_SETUP UNITREE_G1_EXAMPLE_SETUP
    unset UNITREE_G1_SETUP_FILE
    return 1
  fi
  source "${UNITREE_G1_SETUP_FILE}"
done

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# CYCLONEDDS_URI selects eth0 explicitly, so a localhost-only override would
# conflict with the real-robot interface selection.
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'

unset UNITREE_G1_ENV_DIR UNITREE_G1_WS_ROOT UNITREE_G1_ROS2_ROOT
unset UNITREE_G1_ROS_SETUP UNITREE_G1_DDS_SETUP UNITREE_G1_EXAMPLE_SETUP
unset UNITREE_G1_SETUP_FILE
