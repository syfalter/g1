#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
read -r -a includes <<< "$(/usr/bin/python3-config --includes)"
suffix="$(/usr/bin/python3-config --extension-suffix)"
c++ -O2 -std=c++17 -shared -fPIC "${includes[@]}" \
    -I/opt/ros/humble/include/ompl-1.7 -I/usr/include/eigen3 \
    "$root/scripts/task_ompl_route.cpp" \
    -L/opt/ros/humble/lib/x86_64-linux-gnu \
    -Wl,-rpath,/opt/ros/humble/lib/x86_64-linux-gnu -lompl \
    -o "$root/scripts/_task_ompl_route$suffix"
