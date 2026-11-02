#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
build_dir="${HEAD_CAMERA_BUILD_DIR:-${project_dir}/build}"
capture_bin="${build_dir}/src/head_camera_capture"

if [[ ! -x "${capture_bin}" || \
      "${project_dir}/src/head_camera_capture.cpp" -nt "${capture_bin}" || \
      "${project_dir}/src/CMakeLists.txt" -nt "${capture_bin}" ]]; then
    cmake -S "${project_dir}" -B "${build_dir}" -DCMAKE_BUILD_TYPE=Release
    cmake --build "${build_dir}" --target head_camera_capture -j2
fi

cd "${project_dir}"
exec "${capture_bin}" "$@"
