#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$root/scripts/run_task_offline_gallery.sh" --reports \
  "$root/data/offline_task_tests/20260909_194905_936465/report.json" \
  --output "$root/data/offline_task_tests/gallery_straight_20260909" "$@"
