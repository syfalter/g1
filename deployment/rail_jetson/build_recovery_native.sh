#!/usr/bin/env bash
set -e
R="${1:?Pass the Jetson action_runtime_v3 root}"
S="$R/incoming/recovery_20260928/native"
source /opt/ros/foxy/setup.bash
source "$R/native_ws/install/setup.bash"
mkdir -p "$S"
if [ ! -d "$S/source" ]; then
  cp -a "$R/native_ws/src/g1_unitree_control" "$S/source"
fi
cp "$S/arm_sdk_hardware_gateway.cpp" "$S/source/src/arm_sdk_hardware_gateway.cpp"
cp "$S/task_waist_rate.hpp" "$S/source/include/g1_unitree_control/task_waist_rate.hpp"
cmake -S "$S/source" -B "$S/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$S/install"
cmake --build "$S/build" -j2
cd "$S/build"
ctest --output-on-failure
cmake --install "$S/build"
