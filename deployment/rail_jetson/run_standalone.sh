#!/usr/bin/env bash
set -e
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/foxy/setup.bash
source "$root/native_ws/install/setup.bash"
# Foxy's bundled Cyclone 0.7 crashes parsing discovery on this robot network.
# Use the existing Unitree SDK runtime only in this process tree.
dds_lib=/home/unitree/unitree_sdk2-main/thirdparty/lib/aarch64
test -f "$dds_lib/libddsc.so"
export LD_LIBRARY_PATH="$dds_lib:${LD_LIBRARY_PATH:-}"
export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
exec /usr/bin/python3 "$root/deployment/rail_jetson/standalone_supervisor.py" "$@"
