"""Shared camera capture for the recorder and the read-only dashboard."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraSpec:
    name: str
    device: int | str
    width: int
    height: int
    fps: float = 30.0


@dataclass
class CameraFrame:
    rgb: np.ndarray
    jpeg: bytes | None
    captured_monotonic_s: float
    sequence: int


class CameraBroker:
    """Own each physical camera once and expose only its latest captured frame.

    The recorder and dashboard must not each open ``/dev/video*``.  This class
    has one lightweight reader thread per camera, so a slow browser never
    affects the recorder and a video-writing delay never blocks capture.
    """

    def __init__(self, specs: list[CameraSpec], preview_fps: float = 15.0) -> None:
        self._specs = specs
        self._preview_period_s = 1.0 / max(float(preview_fps), 1.0)
        self._frames: dict[str, CameraFrame] = {}
        self._status: dict[str, dict[str, Any]] = {
            spec.name: {
                "name": spec.name,
                "device": str(spec.device),
                "configured": True,
                "opened": False,
                "frame_count": 0,
                "last_frame_monotonic_s": None,
                "last_error": None,
                "width": None,
                "height": None,
            }
            for spec in specs
        }
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._threads: list[threading.Thread] = []
        self._captures: list[cv2.VideoCapture] = []

    @property
    def configured_names(self) -> list[str]:
        return [spec.name for spec in self._specs]

    def start(self) -> dict[str, tuple[int, int]]:
        self._running.set()
        for spec in self._specs:
            thread = threading.Thread(
                target=self._capture_loop,
                args=(spec,),
                name=f"camera-{spec.name}",
                daemon=True,
            )
            self._threads.append(thread)
            thread.start()

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            shapes = self.shapes()
            if len(shapes) == len(self._specs):
                return shapes
            time.sleep(0.02)
        return self.shapes()

    def _capture_loop(self, spec: CameraSpec) -> None:
        cap = cv2.VideoCapture(spec.device)
        with self._lock:
            self._captures.append(cap)
        if not cap.isOpened():
            with self._lock:
                self._status[spec.name]["last_error"] = "open failed"
            return

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, spec.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, spec.height)
        cap.set(cv2.CAP_PROP_FPS, spec.fps)
        last_jpeg_time = 0.0
        sequence = 0

        while self._running.is_set():
            ok, bgr = cap.read()
            now = time.monotonic()
            if not ok:
                with self._lock:
                    self._status[spec.name]["last_error"] = "frame read failed"
                time.sleep(0.02)
                continue

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            jpeg: bytes | None = None
            if now - last_jpeg_time >= self._preview_period_s:
                encoded, buffer = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if encoded:
                    jpeg = buffer.tobytes()
                    last_jpeg_time = now
            sequence += 1
            with self._lock:
                previous = self._frames.get(spec.name)
                if jpeg is None and previous is not None:
                    jpeg = previous.jpeg
                self._frames[spec.name] = CameraFrame(rgb, jpeg, now, sequence)
                status = self._status[spec.name]
                status.update(
                    {
                        "opened": True,
                        "frame_count": sequence,
                        "last_frame_monotonic_s": now,
                        "last_error": None,
                        "width": int(bgr.shape[1]),
                        "height": int(bgr.shape[0]),
                    }
                )

        cap.release()

    def get_rgb(self, name: str) -> np.ndarray | None:
        with self._lock:
            frame = self._frames.get(name)
            return None if frame is None else frame.rgb.copy()

    def get_preview(self, name: str) -> tuple[bytes, dict[str, Any]] | None:
        with self._lock:
            frame = self._frames.get(name)
            if frame is None or frame.jpeg is None:
                return None
            return frame.jpeg, {
                "sequence": frame.sequence,
                "captured_monotonic_s": frame.captured_monotonic_s,
            }

    def shapes(self) -> dict[str, tuple[int, int]]:
        with self._lock:
            return {
                name: (int(status["width"]), int(status["height"]))
                for name, status in self._status.items()
                if status["width"] is not None and status["height"] is not None
            }

    def status(self) -> dict[str, dict[str, Any]]:
        now = time.monotonic()
        with self._lock:
            result = {name: dict(value) for name, value in self._status.items()}
        for value in result.values():
            timestamp = value.get("last_frame_monotonic_s")
            value["age_ms"] = None if timestamp is None else round((now - timestamp) * 1000.0, 1)
            value["healthy"] = bool(value["opened"] and value["age_ms"] is not None and value["age_ms"] < 1000)
        return result

    def close(self) -> None:
        self._running.clear()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
