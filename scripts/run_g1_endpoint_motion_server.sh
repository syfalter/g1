#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 NETWORK_INTERFACE [--rviz] [--arm left|right]" >&2
  exit 1
fi

network_interface="$1"
use_rviz=false
active_arm=right
shift
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rviz) use_rviz=true; shift ;;
    --arm)
      if [[ $# -lt 2 || ( "$2" != left && "$2" != right ) ]]; then
        echo "--arm requires left or right" >&2; exit 1
      fi
      active_arm="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

if ! ip link show dev "$network_interface" >/dev/null 2>&1; then
  echo "Network interface '$network_interface' is unavailable." >&2
  exit 2
fi

exec 8>"/tmp/g1-arm-sdk-owner-${UID}.lock"
if ! flock -n 8; then
  echo "Another guarded arm controller owns the local lock; refusing to start endpoint control." >&2
  exit 2
fi

source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
source /home/syf/g1_upper_body_ws/install/setup.bash
set -u
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${network_interface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

launch_pid=""
cleanup_done=false
cleanup() {
  if [[ "$cleanup_done" == true ]]; then
    return
  fi
  cleanup_done=true
  set +e
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    echo
    echo "Stopping endpoint motion server..."
    timeout 5 ros2 service call /g1/arm_gateway/disarm std_srvs/srv/Trigger '{}' >/dev/null 2>&1
    sleep 3
    kill -INT "$launch_pid" 2>/dev/null
    wait "$launch_pid" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

ros2 launch g1_upper_body_moveit_config real_shadow_moveit.launch.py \
  active_arm:="$active_arm" \
  use_rviz:="$use_rviz" \
  show_planned_overlay:="$use_rviz" \
  start_shadow_executor:=false \
  enable_rviz_interactive_execution:="$use_rviz" \
  start_hardware_gateway:=true \
  hardware_output_enabled:=true \
  hardware_authorization:=I_UNDERSTAND_ARM_SDK_OUTPUT &
launch_pid=$!

echo "Waiting for guarded gateway and endpoint-pose service..."
for _ in $(seq 1 30); do
  if timeout 2 ros2 service type /g1/arm_gateway/arm >/dev/null 2>&1 && \
     timeout 2 ros2 service type /move_single_arm >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

if ! timeout 2 ros2 service type /g1/arm_gateway/arm >/dev/null 2>&1 || \
   ! timeout 2 ros2 service type /move_single_arm >/dev/null 2>&1; then
  echo "Startup failed: required services did not appear." >&2
  exit 3
fi

echo
echo "PREARM READY"
echo "  Core API: /move_single_arm (g1_upper_body_moveit_config/srv/PlanSingleArm)"
echo "  execute=false plans only; execute=true plans and executes through the guarded gateway."
if [[ "$use_rviz" == true ]]; then
  echo "  RViz Plan and Execute are enabled through the same guarded trajectory gateway."
fi
echo "  L2+B remains the native emergency damping command."
echo "  Selected arm: $active_arm. Keep it untouched during takeover and feedforward blending."
if [[ "$active_arm" == left ]]; then
  echo "  LEFT TEST uses an unvalidated copied payload seed. Complete admittance testing first."
fi
if [[ "$use_rviz" == true ]]; then
  echo "  Use the G1 Endpoint panel in RViz to ARM once, monitor status, or Disarm."
  echo
  echo "ENDPOINT MOTION SERVER ACTIVE - UNARMED"
  echo "Press Ctrl+C for controlled release and shutdown."
  wait "$launch_pid"
  exit 0
fi

read -r -p "Type ARM once to take over and hold the current upper-body pose: " confirmation
if [[ "$confirmation" != "ARM" ]]; then
  echo "Not armed; exiting."
  exit 4
fi

arm_response="$(timeout 12 ros2 service call /g1/arm_gateway/arm std_srvs/srv/Trigger '{}')"
echo "$arm_response"
if [[ "$arm_response" != *"success=True"* ]]; then
  echo "Arm takeover was rejected; exiting." >&2
  exit 5
fi

echo "Waiting 5 s for anchored hold and gravity-feedforward blending..."
sleep 5

echo
echo "ENDPOINT MOTION SERVER ACTIVE"
echo "Pose requests now explicitly choose plan-only or execute."
echo "Press Ctrl+C for controlled release and shutdown."
wait "$launch_pid"
