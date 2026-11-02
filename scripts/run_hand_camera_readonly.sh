#!/usr/bin/env bash
set -eo pipefail
network="${1:?network interface}"
output="${2:?new output directory}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk="/home/syf/unitree_sdk2"
binary="$root/build/probe_hand_camera_readonly"
g++ -std=c++17 -O2 -pthread "$root/scripts/probe_hand_camera_readonly.cpp" \
  -I"$root/third_party/g1-rh56dfx-control/src" -I"$sdk/include" \
  -I"$sdk/thirdparty/include" -I"$sdk/thirdparty/include/ddscxx" \
  "$sdk/lib/x86_64/libunitree_sdk2.a" -L"$sdk/thirdparty/lib/x86_64" \
  -Wl,-rpath,"$sdk/thirdparty/lib/x86_64" -lddscxx -lddsc -o "$binary"
# Do not mix the SDK C++ DDS library with ROS's different C DDS version.
exec env LD_LIBRARY_PATH="$sdk/thirdparty/lib/x86_64" "$binary" "$network" "$output"
