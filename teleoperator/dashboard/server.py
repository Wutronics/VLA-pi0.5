"""Read-only local dashboard server.

It receives lossy telemetry from the collection process over localhost ZMQ and
never imports, calls, or controls the D3 SDK.
"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
import threading
import time
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse


class DashboardStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {"collector": {"connected": False}}
        self._frames: dict[str, tuple[bytes, dict[str, Any]]] = {}
        self._stop = threading.Event()

    def update_status(self, status: dict[str, Any]) -> None:
        with self._lock:
            self._status = status

    def update_frame(self, name: str, jpeg: bytes, metadata: dict[str, Any]) -> None:
        with self._lock:
            self._frames[name] = (jpeg, metadata)

    def status(self) -> dict[str, Any]:
        with self._lock:
            result = json.loads(json.dumps(self._status))
        last = result.get("published_at_unix_s")
        result.setdefault("collector", {})["connected"] = bool(last and time.time() - last < 2.0)
        result["collector"]["age_ms"] = None if not last else round((time.time() - last) * 1000.0, 1)
        return result

    def frame(self, name: str) -> tuple[bytes, dict[str, Any]] | None:
        with self._lock:
            return self._frames.get(name)

    def stop(self) -> None:
        self._stop.set()


def _subscriber_loop(endpoint: str, store: DashboardStore) -> None:
    import zmq

    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVHWM, 2)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.connect(endpoint)
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    try:
        while not store._stop.is_set():
            if not poller.poll(250):
                continue
            parts = socket.recv_multipart()
            if len(parts) < 2:
                continue
            topic, header = parts[0], json.loads(parts[1].decode("utf-8"))
            if topic == b"status":
                store.update_status(header)
            elif topic == b"frame" and len(parts) == 3:
                store.update_frame(str(header.get("camera_name", "unknown")), parts[2], header)
    finally:
        socket.close(linger=0)


def create_app(endpoint: str) -> FastAPI:
    app = FastAPI(title="D3 Collection Dashboard", docs_url=None, redoc_url=None)
    store = DashboardStore()
    static = Path(__file__).resolve().parent / "static"

    @app.on_event("startup")
    def start_subscriber() -> None:
        app.state.store = store
        app.state.subscriber = threading.Thread(
            target=_subscriber_loop,
            args=(endpoint, store),
            name="dashboard-zmq-subscriber",
            daemon=True,
        )
        app.state.subscriber.start()

    @app.on_event("shutdown")
    def stop_subscriber() -> None:
        store.stop()
        app.state.subscriber.join(timeout=1.0)

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(static / "index.html")

    @app.get("/app.js")
    def app_js() -> FileResponse:
        return FileResponse(static / "app.js", media_type="application/javascript")

    @app.get("/style.css")
    def style_css() -> FileResponse:
        return FileResponse(static / "style.css", media_type="text/css")

    @app.get("/api/status")
    def api_status() -> dict[str, Any]:
        return store.status()

    @app.get("/stream/{camera_name}")
    async def stream(camera_name: str) -> StreamingResponse:
        if store.frame(camera_name) is None:
            raise HTTPException(status_code=404, detail="camera has not published a frame")

        async def mjpeg():
            last_sequence = None
            while True:
                item = store.frame(camera_name)
                if item is not None:
                    jpeg, metadata = item
                    sequence = metadata.get("sequence")
                    if sequence != last_sequence:
                        last_sequence = sequence
                        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                await asyncio.sleep(1.0 / 20.0)

        return StreamingResponse(mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.websocket("/ws/status")
    async def ws_status(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(store.status())
                await asyncio.sleep(0.25)
        except WebSocketDisconnect:
            return

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="D3 collection read-only dashboard")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:8766")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    import uvicorn

    uvicorn.run(create_app(args.endpoint), host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
