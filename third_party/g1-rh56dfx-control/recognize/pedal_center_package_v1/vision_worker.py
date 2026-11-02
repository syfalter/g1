#!/usr/bin/env python3
"""Unix-socket worker used by the C++ vision_service RPC facade."""

from __future__ import annotations

import argparse
import json
import os
import signal
import socketserver
import threading
from pathlib import Path
from typing import Any

from vision_runtime import (
    DEFAULT_CAPTURES_DIR,
    DEFAULT_HEAD_SERIAL,
    PedalVisionRuntime,
    VisionConfig,
)

OK = 0
INVALID_ARGUMENT = 100
NO_TARGET = 102
INVALID_DEPTH = 103
TARGET_FRAME_UNAVAILABLE = 104
INTERNAL_ERROR = 200
MAX_REQUEST_BYTES = 64 * 1024


class RequestError(ValueError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, required=True)
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


def _request_id(request: dict[str, Any]) -> int:
    value = request.get("request_id", 0)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise RequestError("request_id must be a non-negative integer")
    return value


def _error(code: int, message: str, request_id: int = 0) -> dict[str, Any]:
    return {"code": code, "message": message, "request_id": request_id}


def _recognize_options(request: dict[str, Any]) -> tuple[bool, bool, bool]:
    values = (
        request.get("require_3d", True),
        request.get("require_target_frame", False),
        request.get("save_artifacts", False),
    )
    if not all(isinstance(value, bool) for value in values):
        raise RequestError("recognition options must be boolean")
    return values


def _observation_reply(
    request: dict[str, Any],
    observation: dict[str, Any],
) -> dict[str, Any]:
    request_id = _request_id(request)
    require_3d, require_target_frame, _ = _recognize_options(request)
    reply = dict(observation)
    reply.update({"code": OK, "message": "ok", "request_id": request_id})
    if not reply["valid_2d"]:
        reply.update(code=NO_TARGET, message="pedal target was not detected")
    elif require_3d and not reply["valid_3d"]:
        reply.update(code=INVALID_DEPTH, message="operation point has no valid depth")
    elif require_target_frame and not reply["target_frame_ready"]:
        reply.update(code=TARGET_FRAME_UNAVAILABLE, message="target-frame transform is not configured")

    return reply


class WorkerApplication:
    def __init__(self, runtime: PedalVisionRuntime):
        self.runtime = runtime

    def handle(self, envelope: dict[str, Any]) -> dict[str, Any]:
        action = envelope.get("action")
        request = envelope.get("request", {})
        if not isinstance(request, dict):
            return _error(INVALID_ARGUMENT, "request must be a JSON object")
        try:
            if action == "get_status":
                request_id = _request_id(request)
                return {
                    "code": OK,
                    "message": "ok",
                    "request_id": request_id,
                    **self.runtime.status(),
                }
            if action == "recognize_once":
                _request_id(request)
                _, _, save_artifacts = _recognize_options(request)
                restart_stream = request.get("restart_stream", False)
                if not isinstance(restart_stream, bool):
                    raise RequestError("restart_stream must be boolean")
                if restart_stream:
                    observation = self.runtime.capture_once(save_artifacts=save_artifacts,
                        restart_stream=True, request_id=_request_id(request))
                else:
                    observation = self.runtime.capture_once(save_artifacts=save_artifacts)
                return _observation_reply(request, observation)
            return _error(INVALID_ARGUMENT, f"unknown action: {action!r}")
        except RequestError as error:
            return _error(INVALID_ARGUMENT, str(error), request.get("request_id", 0))
        except (OSError, RuntimeError, ValueError) as error:
            return _error(INTERNAL_ERROR, str(error), request.get("request_id", 0))


class _ThreadingUnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


class _RequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        raw = self.rfile.readline(MAX_REQUEST_BYTES + 1)
        if len(raw) > MAX_REQUEST_BYTES:
            response = _error(INVALID_ARGUMENT, "request is too large")
        else:
            try:
                envelope = json.loads(raw.decode("utf-8"))
                if not isinstance(envelope, dict):
                    raise ValueError("request envelope must be a JSON object")
                response = self.server.application.handle(envelope)  # type: ignore[attr-defined]
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                response = _error(INVALID_ARGUMENT, f"invalid request: {error}")
        self.wfile.write(json.dumps(response, separators=(",", ":")).encode("utf-8") + b"\n")


def main() -> int:
    args = parse_args()
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
    socket_path = args.socket.expanduser().resolve()
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    if socket_path.exists():
        socket_path.unlink()

    with PedalVisionRuntime(config, rs) as runtime:
        with _ThreadingUnixServer(str(socket_path), _RequestHandler) as server:
            server.application = WorkerApplication(runtime)  # type: ignore[attr-defined]
            os.chmod(socket_path, 0o660)

            def stop_server(*_: Any) -> None:
                threading.Thread(target=server.shutdown, daemon=True).start()

            signal.signal(signal.SIGINT, stop_server)
            signal.signal(signal.SIGTERM, stop_server)
            try:
                server.serve_forever(poll_interval=0.2)
            finally:
                socket_path.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
