#!/usr/bin/env bash
set -e
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
sdk=/home/unitree/unitree_sdk2-main
mkdir -p "$root/build"
g++ -std=c++17 -O2 -pthread "$root/scripts/task_dds_observer.cpp" -I"$sdk/include" -I"$sdk/thirdparty/include" -I"$sdk/thirdparty/include/ddscxx" "$sdk/lib/aarch64/libunitree_sdk2.a" -L"$sdk/thirdparty/lib/aarch64" -Wl,--disable-new-dtags,-rpath,"$sdk/thirdparty/lib/aarch64" -lddscxx -lddsc -o "$root/build/task_dds_observer"
