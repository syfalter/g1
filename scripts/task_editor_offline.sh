#!/usr/bin/env bash
# Separate service: never delegates to the hardware lifecycle.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-start}"
shift || true
if [[ $# -ne 0 ]]; then
  printf '%s\n' 'Usage: task_editor_offline.sh {start|stop|run|status}' >&2
  exit 2
fi
unit=g1-trajectory-editor-offline.service
case "$action" in
  start)
    /usr/bin/python3 -c "import sys; sys.path.insert(0, '$root/scripts'); from task_console_handoff import acquire_console_lock; acquire_console_lock().close()"
    env_args=()
    for key in DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS QT_QPA_PLATFORM; do
      if [[ -v "$key" ]]; then env_args+=("--setenv=$key=${!key}"); fi
    done
    systemd-run --user --collect --unit="$unit" \
      --property=KillMode=mixed --property=TimeoutStopSec=120 \
      "${env_args[@]}" -- /bin/bash "$root/scripts/task_editor_offline.sh" run
    ;;
  stop) systemctl --user stop "$unit" ;;
  status) systemctl --user status "$unit" ;;
  run)
    source /opt/ros/humble/setup.bash
    source "$root/install/setup.bash"
    export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    # ROS_LOCALHOST_ONLY supplies the loopback interface itself.
    # WSL mirrored loopback can traverse a 1500-byte interface. Use DDS
    # fragmentation instead of IP fragmentation for robot states and TF.
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1024B</FragmentSize></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
    unset ROS_DISCOVERY_SERVER G1_DDS_INTERFACE_ADDRESS
    export OPENBLAS_NUM_THREADS=1
    exec /usr/bin/python3 "$root/scripts/task_editor_offline.py"
    ;;
  *) printf '%s\n' 'Unknown offline editor action' >&2; exit 2 ;;
esac
