#!/usr/bin/env python3
import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import cv2
except ImportError:
    print("record_right_wrist_camera: python3-opencv is not installed", file=sys.stderr)
    raise SystemExit(2)


DEFAULT_DEVICE = (
    "/dev/v4l/by-id/"
    "usb-Intel_R__RealSense_TM__Depth_Camera_405_"
    "Intel_R__RealSense_TM__Depth_Camera_405_262723071532-video-index4"
)


def gstreamer_quote(value):
    return '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'


def create_h264_writer(path, fps, width, height):
    pipeline = (
        "appsrc ! videoconvert ! video/x-raw,format=I420 ! "
        "x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 "
        "key-int-max=30 ! video/x-h264,profile=baseline ! "
        "h264parse ! mp4mux faststart=true ! "
        f"filesink location={gstreamer_quote(path)}"
    )
    return cv2.VideoWriter(
        pipeline, cv2.CAP_GSTREAMER, 0, fps, (width, height), True
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record MP4 video from the G1 right-wrist D405 color camera."
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="output MP4 path (default: captures/right_wrist_TIMESTAMP.mp4)",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=10.0,
        help="recording duration in seconds; 0 records until Ctrl+C (default: 10)",
    )
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help="override the right-wrist V4L2 device path",
    )
    args = parser.parse_args()
    if args.duration < 0:
        parser.error("--duration must be zero or greater")
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        args.output = Path("captures") / f"right_wrist_{timestamp}.mp4"
    if args.output.suffix.lower() != ".mp4":
        parser.error("--output must end in .mp4")
    return args


def record(args):
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.stem}.recording{output.suffix}")
    if output.exists():
        raise RuntimeError(f"refusing to overwrite existing file: {output}")
    if temporary.exists():
        raise RuntimeError(f"temporary file already exists: {temporary}")

    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    writer = None
    completed = False
    frame_count = 0
    started = None
    failure = None

    try:
        if not camera.isOpened():
            raise RuntimeError(f"cannot open right-wrist camera: {args.device}")

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 30)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ok, frame = camera.read()
        if not ok or frame is None:
            raise RuntimeError("right-wrist camera opened but returned no color frame")

        height, width = frame.shape[:2]
        fps = camera.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            fps = 30.0
        writer = create_h264_writer(temporary, fps, width, height)
        if not writer.isOpened():
            raise RuntimeError(
                "cannot initialize the H.264 MP4 encoder; check the GStreamer "
                "x264enc, h264parse, and mp4mux plugins"
            )

        duration_text = (
            "until Ctrl+C" if args.duration == 0 else f"for {args.duration:g} seconds"
        )
        print(f"Recording {width}x{height} at {fps:g} FPS {duration_text}...")

        started = time.monotonic()
        consecutive_failures = 0
        while True:
            writer.write(frame)
            frame_count += 1

            if args.duration > 0 and time.monotonic() - started >= args.duration:
                break

            ok, frame = camera.read()
            if ok and frame is not None:
                consecutive_failures = 0
                continue
            consecutive_failures += 1
            if consecutive_failures >= 30:
                raise RuntimeError("camera stopped returning frames")

        completed = True
    except KeyboardInterrupt:
        completed = frame_count > 0
        print("\nStopping recording...")
    except Exception as error:
        failure = error
    finally:
        camera.release()
        if writer is not None:
            writer.release()

    if failure is not None:
        if temporary.exists():
            temporary.unlink()
        raise failure
    if not completed:
        if temporary.exists():
            temporary.unlink()
        raise RuntimeError("recording did not produce any frames")

    temporary.rename(output)
    elapsed = time.monotonic() - started
    print(f"Saved {frame_count} frames ({elapsed:.1f} s): {output}")


def main():
    try:
        record(parse_args())
    except RuntimeError as error:
        print(f"record_right_wrist_camera: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
