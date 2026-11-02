#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python="${G1_TEST_PYTHON:-/usr/bin/python3}"
if [[ $# -ne 0 ]]; then
  printf '%s\n' 'Usage: bash scripts/run_onboarding_checks.sh' >&2
  exit 2
fi

cd "$root/scripts"
printf '%s\n' 'Offline onboarding checks: synthetic inputs, no ROS/DDS or robot I/O.'
# Keep a reviewed allowlist: the wider test tree also needs ROS and local data.
exec "$python" -m unittest -v \
  test_task_trajectory \
  test_task_future \
  test_task_model_resources \
  test_task_runtime_provenance \
  test_task_contact_surface \
  test_task_preview_height \
  test_task_session
