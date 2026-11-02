#!/usr/bin/env python3
"""Command-line adapter for the reusable pedal-center recognizer."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from pedal_center_recognizer import DEFAULT_STAGE1, DEFAULT_STAGE2, PedalCenterRecognizer


def main() -> None:
    parser = argparse.ArgumentParser(description="回正轮盘端到端推理：两框、圆心、操作区和操作点")
    parser.add_argument("--source", type=Path, required=True, help="单张图片或非递归图片目录")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage1", type=Path, default=DEFAULT_STAGE1)
    parser.add_argument("--stage2", type=Path, default=DEFAULT_STAGE2)
    parser.add_argument("--device", default="0")
    parser.add_argument("--stage1-conf", type=float, default=0.10)
    parser.add_argument("--stage2-conf", type=float, default=0.001)
    parser.add_argument("--padding", type=float, default=0.15)
    parser.add_argument("--radial-offset-scale", type=float, default=1.35)
    args = parser.parse_args()
    paths = [args.source] if args.source.is_file() else sorted(path for path in args.source.iterdir() if path.suffix.lower() in {".png", ".jpg", ".jpeg"})
    if not paths:
        raise SystemExit("没有可推理图片")
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    recognizer = PedalCenterRecognizer(
        args.stage1, args.stage2, device=args.device, stage1_conf=args.stage1_conf,
        stage2_conf=args.stage2_conf, crop_padding=args.padding,
        radial_offset_scale=args.radial_offset_scale,
    )
    predictions = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        result = recognizer.predict(image)
        payload = result.to_dict()
        payload["image"] = path.name
        cv2.imwrite(str(output / path.name), recognizer.annotate(image, result))
        predictions.append(payload)
    (output / "predictions.json").write_text(json.dumps(predictions, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"完成：{len(predictions)} 张 -> {output}")


if __name__ == "__main__":
    main()
