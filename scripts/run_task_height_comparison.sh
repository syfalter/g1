#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$root/scripts/run_task_offline_gallery.sh" --reports \
  "$root/data/offline_task_tests/20260909_185831_614140/report.json" \
  "$root/data/offline_task_tests/20260909_185921_540983/report.json" \
  "$root/data/offline_task_tests/20260909_190120_118206/report.json" \
  "$root/data/offline_task_tests/20260909_184753_460750/report.json" \
  --output "$root/data/offline_task_tests/gallery_height_20260909" "$@"
