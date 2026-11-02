#!/usr/bin/env bash
set -eo pipefail
network="${1:?NETWORK_INTERFACE required}"
output="${2:?NEW_JSON_PATH required}"
shift 2
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk=/home/syf/unitree_sdk2
ip link show dev "$network" >/dev/null
g++ -std=c++17 -O2 -pthread "$root/scripts/probe_task_vision.cpp" \
  -I"$root/third_party/g1-rh56dfx-control/src" -I"$sdk/include" \
  -I"$sdk/thirdparty/include" -I"$sdk/thirdparty/include/ddscxx" \
  "$sdk/lib/x86_64/libunitree_sdk2.a" -L"$sdk/thirdparty/lib/x86_64" \
  -Wl,-rpath,"$sdk/thirdparty/lib/x86_64" -lddscxx -lddsc \
  -o "$root/build/probe_task_vision"
exec /usr/bin/python3 - "$root" "$network" "$output" "$@" <<'PY'
import os
from pathlib import Path
import sys
root=Path(sys.argv[1])
sys.path.insert(0,str(root/'scripts'))
from task_sdk_environment import sdk_environment
peer=os.environ.get('G1_VISION_DDS_PEER','192.168.123.164')
environment=sdk_environment(peer if os.environ.get('CYCLONEDDS_URI') else None)
binary=str(root/'build/probe_task_vision')
os.execve(binary,[binary,*sys.argv[2:]],environment)
PY
