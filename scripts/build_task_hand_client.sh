#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk=/home/syf/unitree_sdk2
mkdir -p "$root/build"
g++ -std=c++17 -O2 -pthread "$root/scripts/task_hand_client.cpp" \
  -I"$root/third_party/g1-rh56dfx-control/src" -I"$sdk/include" \
  -I"$sdk/thirdparty/include" -I"$sdk/thirdparty/include/ddscxx" \
  "$sdk/lib/x86_64/libunitree_sdk2.a" -L"$sdk/thirdparty/lib/x86_64" \
  -Wl,--disable-new-dtags,-rpath,"$sdk/thirdparty/lib/x86_64" -lddscxx -lddsc \
  -o "$root/build/task_hand_client"
