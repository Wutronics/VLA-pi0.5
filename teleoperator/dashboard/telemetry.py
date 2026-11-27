"""Small, lossy IPC channel for collector telemetry.

The collector is always the producer.  Dashboard consumers are deliberately
best-effort: a slow browser or a stopped dashboard must never back-pressure
the robot-control loop.
"""

from __future__ import annotations

import json
import time
from typing import Any


def _json_default(value: Any) -> Any:
    """Convert common numeric containers without importing NumPy as a hard dependency."""
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "item"):
        return value.item()
    raise TypeError(f"{type(value).__name__} is not JSON serializable")


class TelemetryPublisher:
    """Publish latest telemetry with a one-message high-water mark.

    ``send_multipart(..., NOBLOCK)`` is intentional.  Monitoring data may be
    dropped, while robot-control and dataset-writing data may not be delayed.
    """

    def __init__(self, endpoint: str) -> None:
        try:
            import zmq
        except ImportError as exc:  # pragma: no cover - installation error path
            raise RuntimeError("dashboard requires pyzmq") from exc

        self._zmq = zmq
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, 1)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(endpoint)
        self.endpoint = endpoint
        self.dropped_messages = 0

    @staticmethod
    def _encode(payload: dict[str, Any]) -> bytes:
        envelope = {"published_at_unix_s": time.time(), **payload}
        return json.dumps(envelope, default=_json_default, separators=(",", ":")).encode("utf-8")

    def _send(self, parts: list[bytes]) -> None:
        try:
            self._socket.send_multipart(parts, flags=self._zmq.NOBLOCK)
        except self._zmq.Again:
            self.dropped_messages += 1

    def publish_status(self, payload: dict[str, Any]) -> None:
        self._send([b"status", self._encode(payload)])

    def publish_frame(self, camera_name: str, jpeg: bytes, metadata: dict[str, Any]) -> None:
        header = {"camera_name": camera_name, **metadata}
        self._send([b"frame", self._encode(header), jpeg])

    def close(self) -> None:
        self._socket.close(linger=0)
