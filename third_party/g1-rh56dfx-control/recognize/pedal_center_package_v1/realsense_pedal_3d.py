#!/usr/bin/env python3
"""Capture one synchronized RGB-D observation using the shared vision runtime."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from vision_runtime import (
    DEFAULT_CAPTURES_DIR,
    DEFAULT_HEAD_SERIAL,
    PedalVisionRuntime,
    VisionConfig,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default=DEFAULT_HEAD_SERIAL)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument("--device", default="0")
    parser.add_argument("--depth-patch-radius", type=int, default=2)
    parser.add_argument("--stage1", type=Path)
    parser.add_argument("--stage2", type=Path)
    parser.add_argument("--target-extrinsics", type=Path)
    parser.add_argument("--captures-dir", type=Path, default=DEFAULT_CAPTURES_DIR)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        import pyrealsense2 as rs

        defaults = VisionConfig()
        config = VisionConfig(
            serial=args.serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            warmup_frames=args.warmup_frames,
            device=args.device,
            depth_patch_radius=args.depth_patch_radius,
            stage1_path=defaults.stage1_path if args.stage1 is None else args.stage1,
            stage2_path=defaults.stage2_path if args.stage2 is None else args.stage2,
            target_extrinsics=args.target_extrinsics,
            captures_dir=args.captures_dir,
        )
        with PedalVisionRuntime(config, rs) as runtime:
            observation = runtime.capture_once(save_artifacts=True)
        print(json.dumps(observation, ensure_ascii=False, indent=2))
        return 0
    except (RuntimeError, ValueError, OSError, json.JSONDecodeError) as error:
        print(f"realsense_pedal_3d: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
