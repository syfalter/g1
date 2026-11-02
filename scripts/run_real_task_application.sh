#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
network="${1:-eth0}"
if [[ $# -gt 0 ]]; then shift; fi
hardware=false
connect_vision=false
extra=()
hand_control=true
waist_assist=true
dring_profile=steep_straight
planning_config="$root/config/task_planning.yaml"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --hardware) hardware=true; shift ;;
    --managed-lifecycle) extra+=(--managed-lifecycle); shift ;;
    --dring-contact-test) extra+=(--dring-contact-test); shift ;;
    --connect-vision) connect_vision=true; shift ;;
    --ready-poses) extra+=(--ready-poses "$2"); shift 2 ;;
    --saved-test-archive) extra+=(--saved-test-archive "$2"); shift 2 ;;
    --waist-assist) waist_assist=true; shift ;;
    --no-waist-assist) waist_assist=false; shift ;;
    --dring-profile) dring_profile="$2"; shift 2 ;;
    --planning-config) planning_config="$2"; extra+=(--planning-config "$2"); shift 2 ;;
    --legacy-fixed-hand) hand_control=false; dring_profile=legacy; shift ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done
if [[ "$dring_profile" != steep_straight && "$dring_profile" != legacy ]]; then
  printf '%s\n' 'Unknown D-ring profile' >&2; exit 2
fi
extra+=(--dring-profile "$dring_profile")
if [[ "$waist_assist" == true ]]; then extra+=(--waist-assist); fi
if [[ "$hand_control" == true ]]; then
  bash "$root/scripts/build_task_hand_client.sh"
  extra+=(--hand-control)
fi
ip link show dev "$network" >/dev/null
offline_ui=false
if [[ "$(<"/sys/class/net/$network/operstate")" != up ]]; then
  if [[ "$hardware" == true || "$connect_vision" == true ]]; then
    printf '%s\n' "Interface $network is not UP; refusing hardware/vision connection." >&2
    exit 2
  fi
  offline_ui=true
  extra+=(--disconnected)
  printf '%s\n' 'Network disconnected: isolated UI only, no historical pose substituted for live state.'
fi
exec 8>"/tmp/g1-arm-sdk-owner-${UID}.lock"
if ! flock -n 8; then
  printf '%s\n' 'Another arm controller owns this machine. Refusing a second owner.' >&2
  exit 2
fi
source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
source "$root/install/setup.bash"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${network}\" priority=\"default\" multicast=\"default\" /></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
if [[ -n "${G1_DDS_INTERFACE_ADDRESS:-}" && "$offline_ui" == false ]]; then
  ip -j -4 address show dev "$network" | python3 -c '
import ipaddress, json, sys
address = str(ipaddress.IPv4Address(sys.argv[1]))
if not any(a["local"] == address for link in json.load(sys.stdin) for a in link["addr_info"]):
    sys.exit("Requested DDS IPv4 address is not assigned to this interface")
' "$G1_DDS_INTERFACE_ADDRESS"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"${G1_DDS_INTERFACE_ADDRESS}\" priority=\"default\" multicast=\"default\" /></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
fi
if [[ "$offline_ui" == true ]]; then
  export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
  export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
fi
runtime="$(mktemp -d --tmpdir g1-real-task-XXXXXX)"
launch_pid=''
app_pid=''
cleanup() {
  trap - EXIT INT TERM
  if [[ "$hardware" == true && -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    timeout 3 ros2 service call /g1/arm_gateway/stop_task std_srvs/srv/Trigger '{}' || true
    printf '%s\n' 'Controller retained for support. Unload arms and use the robot shutdown procedure.'
    printf '%s\n' "Launch PID: $launch_pid; no automatic disarm or weight release."
    # Keep the supervisor alive; exiting a systemd unit would kill its children.
    wait "$launch_pid" 2>/dev/null || true
    return
  fi
  if [[ -n "$app_pid" ]]; then kill -TERM "$app_pid" 2>/dev/null || true; fi
  if [[ -n "$launch_pid" ]]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ "$connect_vision" == true ]]; then
    ssh -S "$G1_VISION_SSH_CONTROL_PATH" -O exit unitree@192.168.0.249 2>/dev/null || true
  fi
  rm -f "$runtime/robot.urdf"
  rmdir "$runtime" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
if [[ "$connect_vision" == true ]]; then
  export G1_VISION_SSH_CONTROL_PATH="$runtime/vision-%C"
  ssh -M -S "$G1_VISION_SSH_CONTROL_PATH" -o StrictHostKeyChecking=yes \
    -o ConnectTimeout=5 -o ControlPersist=8h -o ServerAliveInterval=30 \
    -o ServerAliveCountMax=3 -fN unitree@192.168.0.249
fi
authorization=OUTPUT_DISABLED
if [[ "$hardware" == true ]]; then
  authorization=I_UNDERSTAND_ARM_SDK_OUTPUT
  extra+=(--hardware)
  printf '%s\n' 'Hardware-enabled commissioning session. Startup does NOT ARM or execute.'
fi
setsid ros2 launch g1_upper_body_moveit_config real_shadow_moveit.launch.py \
  active_arm:=left task_application_enabled:=true \
  task_articulated_hands:="$hand_control" task_waist_enabled:="$waist_assist" \
  task_planning_config:="$planning_config" \
  use_rviz:=true show_planned_overlay:=false start_shadow_executor:=false \
  enable_rviz_interactive_execution:=false start_hardware_gateway:=true \
  hardware_output_enabled:="$hardware" hardware_authorization:="$authorization" \
  export_robot_urdf:="$runtime/robot.urdf" \
  rviz_config:="$root/assets/camera_reference/real_task.rviz" &
launch_pid=$!
for _ in $(seq 1 60); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then exit 3; fi
  if [[ -s "$runtime/robot.urdf" ]]; then break; fi
  sleep 1
done
if [[ ! -s "$runtime/robot.urdf" ]]; then exit 3; fi
/usr/bin/python3 "$root/scripts/real_task_app.py" --network "$network" \
  --robot-urdf "$runtime/robot.urdf" "${extra[@]}" &
app_pid=$!
wait "$app_pid"
