#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
domain=93
seen_domain=false
argv=("$@")
for ((i=0; i<${#argv[@]}; i++)); do
    case "${argv[i]}" in
        --ros-domain-id)
            $seen_domain && { printf '%s\n' 'Duplicate --ros-domain-id' >&2; exit 2; }
            i=$((i+1)); domain="${argv[i]:-}"; seen_domain=true ;;
        --ros-domain-id=*)
            $seen_domain && { printf '%s\n' 'Duplicate --ros-domain-id' >&2; exit 2; }
            domain="${argv[i]#*=}"; seen_domain=true ;;
    esac
done
case "$domain" in
    9[3-9]) ;;
    *) printf '%s\n' 'Offline ROS domain must be in 93..99.' >&2; exit 2 ;;
esac
source /home/syf/unitree_ws/env/setup_unitree_humble_g1.sh
source "$root/install/setup.bash"
export ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$root/scripts/task_test_fastdds.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTDDS_ENVIRONMENT_FILE
unset SKIP_DEFAULT_XML ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE
exec 9>"/tmp/g1-task-offline-${domain}-${UID}.lock"
flock -n 9 || { printf 'Offline replay domain %s is already in use.\n' "$domain" >&2; exit 2; }
exec /usr/bin/python3 "$root/scripts/task_offline_replay.py" "$@"
